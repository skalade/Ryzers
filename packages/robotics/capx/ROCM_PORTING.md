# CaP-X on ROCm — Porting Knowledge Base

This file is the source of truth for everything we have learned about porting [CaP-X](https://github.com/capgym/cap-x) (a CUDA-only research framework for code-as-policy robot manipulation) to AMD Ryzen AI / ROCm. Read this first before touching any of the dependency wiring in `Dockerfile`.

It is intentionally written to outlast the chat session that produced it. New experiments should append findings here.

---

## TL;DR — What works today

The default `ryzers build capx` produces an image that runs the **privileged** Robosuite cube-stack eval end-to-end on ROCm:

| Component                                   | Status     | Notes                                                                                       |
|---------------------------------------------|------------|---------------------------------------------------------------------------------------------|
| `import capx`                               | works      | editable install of cap-x source, no CUDA build steps                                       |
| `torch` (ROCm 7.2 / HIP 7.2 / PyTorch 2.10) | works      | inherited from base image, never shadowed                                                   |
| `robosuite` (uynitsuj fork)                 | works      | pulled from `git+https://github.com/uynitsuj/robosuite.git`, installed with `--no-deps`     |
| `mujoco` (3.8.x, EGL backend)               | works      | `MUJOCO_GL=egl`, `PYOPENGL_PLATFORM=egl`                                                    |
| `pyroki` (CPU JAX 0.4.29)                   | works      | answers `/ik` and `/plan` over HTTP; ~30s URDF cold-start                                   |
| `capx.serving.launch_pyroki_server`         | works      | bound port 8116                                                                             |
| `openrouter_server` LLM proxy (port 8110)   | works      | tested with `openai/gpt-4o-mini` against OpenRouter                                         |
| Robosuite Franka cube-stack 1-trial eval    | works      | `franka_robosuite_cube_stack_privileged.yaml` — gpt-4o-mini one-shot, reward 1.0 in ~44s    |

---

## Reference architecture: CaP-X dependency map

CaP-X is a four-component framework: CaP-Gym (envs), CaP-Bench, CaP-Agent0 (the eval harness), CaP-RL (training). Most of the code is pure Python. The CUDA dependency surface is concentrated in three places:

1. **Perception servers** under `capx/serving/`:
   - `launch_sam3_server.py`             → CUDA C++ extensions (Max-Fu/sam3 fork)
   - `launch_contact_graspnet_server.py` → CUDA C++ extensions (PointNet++ ops)
   - `launch_pyroki_server.py`           → JAX (CPU works fine; ROCm jaxlib also exists)
2. **Optional extras** declared in `pyproject.toml`:
   - `curobo`           → NVIDIA cuRobo (proprietary CUDA kernels)
   - `contactgraspnet`  → CUDA C++ extensions
   - `verl`             → vLLM + flash-attn (CUDA wheels by default)
   - `molmo`            → vLLM + tensorflow + accelerate
3. **Submodules** referenced by `[tool.uv.sources]`:
   - `capx/third_party/sam3`                   → Max-Fu/sam3
   - `capx/third_party/curobo`                 → NVlabs/curobo
   - `capx/third_party/contact_graspnet_pytorch` → uynitsuj fork
   - `capx/third_party/b1k`                    → BEHAVIOR-1K + Isaac Sim (NVIDIA-only)
   - `capx/third_party/LIBERO-PRO`             → libero benchmarks
   - `capx/third_party/verl`                   → Max-Fu/verl
   - `capx/third_party/robosuite`              → uynitsuj/robosuite (the fork we DO want)
   - `capx/third_party/libero_dependencies/robosuite` → Max-Fu/robosuite (egl_context branch)

The "default" perception profile in upstream's YAML configs is `[SAM3, Contact-GraspNet, PyRoKi]`; the "privileged" / "oracle" profile is `[PyRoKi]` only. **Privileged is the only ROCm-runnable profile out of the box.**

### Which YAML configs are ROCm-compatible?

Inspect the `api_servers:` block:

```sh
grep -A1 _target_ env_configs/<task>/<config>.yaml
```

Compatible iff the only `_target_` listed is `capx.serving.launch_pyroki_server.main`. Examples:

- `env_configs/cube_stack/franka_robosuite_cube_stack_privileged.yaml`        works
- `env_configs/cube_stack/franka_robosuite_cube_stack.yaml`                   needs SAM3+GraspNet
- `env_configs/cube_stack/franka_robosuite_cube_stack_reduced_api*.yaml`      needs SAM3+GraspNet
- `env_configs/cube_stack/franka_robosuite_cube_stack_multiturn_*.yaml`       needs SAM3+GraspNet for VDM
- LIBERO suite (`env_configs/libero/`)                                        needs `--extra libero` venv
- BEHAVIOR / R1Pro (`env_configs/r1pro/`)                                     NVIDIA Isaac Sim, hard NO

---

## The eight gotchas we have hit so far

These are the bugs that cost meaningful time. The smoke test now defends against #2, #3, #4, the build itself defends against #5 and #8.

| # | Gotcha                                                              | Symptom                                                                 | Fix                                                          |
|---|---------------------------------------------------------------------|-------------------------------------------------------------------------|--------------------------------------------------------------|
| 1 | `uv sync --extra robosuite` builds sam3 CUDA extensions             | `OSError: CUDA_HOME environment variable is not set`                    | Don't use uv at all; pip install + `-e cap-x --no-deps`      |
| 2 | `PyOpenGL-accelerate==3.1.6` Cython-precompiled for old CPython     | `error: 'PyLongObject' has no member named 'ob_digit'` (Py 3.12)        | Drop both `PyOpenGL-accelerate` pin and `PyOpenGL==3.1.6`    |
| 3 | Older PyOpenGL lacks EGL device extensions on Py 3.12               | `AttributeError: module 'OpenGL.EGL' has no attribute 'EGLDeviceEXT'`   | Pin `PyOpenGL>=3.1.7`                                        |
| 4 | Vanilla robosuite from PyPI                                         | `TypeError: MujocoEnv.step() got unexpected kwarg 'skip_render_images'` | Install uynitsuj/robosuite fork                              |
| 5 | Robosuite fork pulls `mink>=1.0`, which forces numpy 2.x            | `numpy must stay on 1.x for robosuite/mink compatibility, got 2.4.4`    | Install fork with `--no-deps`; add `numba`, `termcolor`      |
| 6 | JAX `>=0.4.30` requires numpy 2.x; pyroki has no upper bound        | numpy gets force-upgraded                                               | Pin `jax<0.4.30, jaxlib<0.4.30` (matches upstream override)  |
| 7 | Default config tries to start three perception servers              | `ModuleNotFoundError: No module named 'sam3' / pyroki / ...`            | Use `_privileged.yaml` (PyRoKi only) or `launch_sam3_server` overlay (T1-B) |
| 8 | Bundled pip install of transformers/accelerate evicts ROCm torch    | `torch==2.8.0+cu128`, `torchvision::nms does not exist` at SAM3 import  | Split pip install: bulk deps; then transformers; then accelerate (separate transactions) |

---

## Porting tiers

These are the units of work for the rest of this document. Each entry is sized so a single agent session can finish it. **We're working through them in order.** Status is updated as we go.

### Tier 1 — low-hanging fruit (single dep, mechanical change)

| ID    | Item                                                | Status      | Pointer                                              |
|-------|-----------------------------------------------------|-------------|------------------------------------------------------|
| T1-A  | `jax[rocm]` for pyroki (faster IK)                  | NOT FEASIBLE TODAY | see "Tier 1.A" below — version conflict with ROCm 7.2  |
| T1-B  | SAM3 ROCm/CPU fallback investigation                | DONE        | shim ships `launch_sam3_server.py` overlay using HF transformers Sam3+Sam2 |
| T1-C  | LIBERO-PRO support                                  | DEFERRED    | blocked on Contact-GraspNet (T3-A); SAM3 unblocked by T1-B         |
| T1-D  | `--use-oracle-code` (no-LLM) eval mode              | DONE        | shipped as `demo.sh` / `ryzers run /ryzers/demo_capx.sh` |

### Tier 2 — medium effort (multi-dep / build-from-source)

| ID    | Item                                                | Status      | Pointer                                              |
|-------|-----------------------------------------------------|-------------|------------------------------------------------------|
| T2-A  | Local LLM backend (vLLM-ROCm or alt)                | RESEARCHED  | see "Tier 2.A" below; multiple paths, none turnkey today |
| T2-B  | flash-attn ROCm (Triton / Composable Kernel)        | DEFERRED    | only valuable if T2-A or verl-RL is wired up; not blocking anyone |

### Tier 3 — hard, risky, or speculative

| ID    | Item                                                | Status      | Pointer                                              |
|-------|-----------------------------------------------------|-------------|------------------------------------------------------|
| T3-A  | contact_graspnet_pytorch with HIP-ported ops        | not started | needs PointNet++ HIP port; check if any fork exists  |
| T3-B  | molmo extra (vllm + tensorflow + accelerate)        | not started | depends on T2-A; tensorflow-rocm exists              |
| T3-C  | verl-based RL training                              | not started | depends on T2-A and T2-B                             |
| T3-D  | full SAM3 with HIP-ported CUDA kernels              | not started | very hard; punt unless someone really needs SAM3     |

### Out of scope (NVIDIA-only software)

- nvidia-curobo (NVIDIA proprietary CUDA kernels)
- BEHAVIOR / b1k / Isaac Sim (NVIDIA Omniverse)

---

## Tier 1.A — jax[rocm] for pyroki  [NOT FEASIBLE TODAY]

### Goal

Replace CPU JAX with ROCm JAX so PyRoKi's IK runs on the iGPU.

### Findings (2026-05-12)

The version matrix doesn't have a working cell for our base image:

| JAX version | ROCm plugin family    | Numpy floor | ROCm 7.2 host? |
|-------------|-----------------------|-------------|----------------|
| 0.4.29      | none (CPU only)       | 1.22        | yes (current)  |
| 0.4.33-0.4.35 | `jax_rocm60_plugin` | 1.22        | NO — wheel hard-imports `libamdhip64.so.6`, base only has `.so.7` |
| 0.6.0+      | `jax_rocm7_plugin`    | 2.0+        | yes for runtime, but force-upgrades numpy off 1.x and risks breaking robosuite/numba/pyrender deps we hand-pinned |

So the only ROCm wheels that match our ROCm 7.2 host are JAX 0.6.0+, which conflict with `numpy<2`. AMD's ROCm 6 wheels (which support older JAX) cannot find `libamdhip64.so.6` on this image and fail import.

### Probe transcript

```
$ pip install jaxlib-0.4.35 jax_rocm60_pjrt-0.4.35 jax_rocm60_plugin-0.4.35
$ python3 -c "import jax"
ImportError: libamdhip64.so.6: cannot open shared object file: No such file or directory
```

### Decision: stay on CPU JAX

- PyRoKi's `/ik` call latency on CPU JAX 0.4.29 was already in the single-digit milliseconds during the cube-stack run (5 successful IK calls in the time it took the LLM to respond once at 3.13s). The wall-clock budget is dominated by the LLM round-trip, not IK.
- The work needed to make GPU JAX pay off — patching numpy 2.x compatibility through the rest of the stack, and validating robosuite/mink/numba all still work — is much larger than the win. Punt unless a future workload actually batches many IK calls on the critical path.

### What would unblock this

- Either: AMD ships an `jax_rocm7_plugin` for an older JAX (<0.4.30) — unlikely, the rocm-jax repo is moving forward with current main JAX only.
- Or: cap-x lifts the `jax<0.4.30` upper bound, which they cannot easily do without dropping numpy 1.x compat across the rest of their pyproject.
- Or: someone vendors ROCm 6 libraries into the base image alongside ROCm 7 (technically possible, very ugly).

None of these are imminent. Revisit if/when AMD's rocm-jax channel publishes a backport for 0.4.x with `jax_rocm7_plugin`.

---

## Tier 1.B — SAM3 ROCm fallback  [DONE]

### Goal

Get SAM3-style text-prompt + point-prompt segmentation working on ROCm without the CUDA C++ extensions Max-Fu/sam3 builds at install time. This unlocks the default `franka_robosuite_cube_stack.yaml` and every LIBERO config (which all reference `launch_sam3_server.main`).

### Findings

The cap-x SAM3 server has an extremely small surface area — only **two endpoints**:

| Endpoint            | Input                                          | Upstream backend                                      |
|---------------------|------------------------------------------------|-------------------------------------------------------|
| `POST /segment`       | `{image_base64, text_prompt}`                  | `Max-Fu/sam3.Sam3Processor.set_text_prompt()` (CUDA)  |
| `POST /segment_point` | `{image_base64, point_coords: [x, y]}`         | `Max-Fu/sam3.Sam3Model.predict_inst()` (CUDA)         |

Both are wrapped in a FastAPI app at `127.0.0.1:8114`. Cap-x's only consumer (`capx/integrations/vision/sam3.py`) just POSTs JSON and parses `mask_base64 / boxes / scores` — there's no client-side dependency on Max-Fu's specific Python API.

So the fix is just to keep the FastAPI app structure but swap the inference engine. HuggingFace `transformers` ships pure-PyTorch Sam3 and Sam2 implementations starting in transformers 5.0:

| Endpoint            | Replacement backend                                  | Weights source                                |
|---------------------|------------------------------------------------------|-----------------------------------------------|
| `POST /segment`       | `transformers.Sam3Model` text-prompt segmentation    | `facebook/sam3` (HF-gated; needs HF auth)     |
| `POST /segment_point` | `transformers.Sam2Model` point-prompt segmentation   | `facebook/sam2.1-hiera-large` (NOT gated)     |

(HF transformers Sam3 doesn't expose a point-prompt API in the same way Max-Fu's fork does, so we delegate `/segment_point` to Sam2 — which has fully featured point prompts. cap-x only uses point prompts for grasp candidate generation, so any SAM that takes a click and returns a mask works.)

### What we shipped

1. **`packages/robotics/capx/launch_sam3_server.py`** — drop-in FastAPI app implementing both endpoints with transformers Sam3+Sam2. Module docstring starts with "ROCm-compatible drop-in replacement..." so the smoke test can assert this file (and not the upstream CUDA fork) is what got loaded.
2. **`Dockerfile` overlay** — `COPY launch_sam3_server.py /ryzers/cap-x/capx/serving/launch_sam3_server.py` after the editable cap-x install so the upstream YAML configs (`_target_: capx.serving.launch_sam3_server.main`) bind to our shim with no config changes.
3. **`Dockerfile` install split** — pip install is now in three transactions instead of one. Reason: when ALL deps go through pip's resolver in a single transaction, the base image's torch (with the local-version `+rocm7.2.2.lw.git40d237bf` PEP 440 tag) gets evicted in favour of `torch==2.8.0+cu128` from PyPI. Splitting (bulk deps; transformers; accelerate) avoids this. Sandbox-confirmed both reproduction and fix.
4. **`packages/robotics/capx/demo_sam3.sh`** — start the shim, ping `/health`, run `/segment_point` against a 96×96 synthetic image (no auth needed), and optionally exercise `/segment` if `HF_TOKEN` is set.
5. **`test.sh`** — additional asserts that the loaded `launch_sam3_server.__doc__` contains "ROCm-compatible" (so the COPY overlay is verified at smoke-test time) and that `Sam3Model` / `Sam2Model` import.

### Verified

- `ryzers run /ryzers/test_capx.sh` → all assertions pass; transformers 5.8.0; torch 2.10.0+rocm7.2.2; numpy 1.26.4.
- `ryzers run /ryzers/demo_capx.sh` → privileged cube-stack oracle eval still passes (T1-D regression OK).
- `ryzers run /ryzers/demo_sam3_capx.sh` (no HF_TOKEN) → `/health` reports the shim is loaded; `/segment_point` returns 3 masks for a synthetic bright square (top score 0.985, masks_shape `[3, 64, 64]`).
- (Not tested in this session: `/segment` text-prompt path, since it requires accepting the SAM3 license on HF and `huggingface-cli login`. The shim's lazy-load + descriptive 503 error makes it diagnose-friendly when weights are missing.)

### Gotchas to remember

- **transformers 5.x must be installed in its own pip transaction.** Bundling it with the bulk install caused pip to evict the base-image torch and replace it with a non-ROCm CUDA wheel (gotcha #8 in this doc's gotcha table; sandbox-reproducible).
- **`accelerate` likewise.** The Sam3 `from_pretrained` chain uses accelerate's device dispatch; without it Sam3Model imports fine but `.to(_DEVICE)` can hit a missing-meta-device path on some weight shards.
- **`launch_sam3_server.py` must use a triple-quoted docstring**, not `#` comments, for the smoke test's `__doc__` assertion to work.
- **`facebook/sam3` is gated**; users must accept the license at https://huggingface.co/facebook/sam3 and `huggingface-cli login` (or pass `HF_TOKEN`) before `/segment` will load. `/segment_point` works without auth because Sam2 weights are open.

### Eighth gotcha — add to the table

| 8 | Bundled pip transaction with transformers/accelerate | torch silently replaced with `2.8.0+cu128` (CUDA, CPU-only) | Split pip install into three transactions: bulk deps, transformers, accelerate |

---

## Tier 1.C — LIBERO-PRO support  [DEFERRED — blocked on T1-B]

### Goal

Run LIBERO benchmark tasks (130 task suites) on this image.

### Findings (2026-05-12)

The robosuite-fork conflict turns out to be the smaller half of the problem. The bigger half: **every shipped LIBERO config — including the `*_privileged.yaml` variant — declares `[SAM3, Contact-GraspNet, PyRoKi]` as its `api_servers` block.** There is no LIBERO equivalent of the cube_stack `_privileged.yaml` that uses PyRoKi only.

Specifically:
- `env_configs/libero/franka_libero_spatial_0.yaml`           → SAM3 + GraspNet + PyRoKi
- `env_configs/libero/franka_libero_spatial_0_privileged.yaml` → SAM3 + GraspNet + PyRoKi (despite the name)
- All other libero configs in this directory                   → same pattern

So even if we sort out the dual-robosuite-fork installation, we cannot run any of the shipped LIBERO configs on ROCm without writing our own from scratch.

The robosuite-fork side of it (Max-Fu/robosuite#maxf/egl_context vs uynitsuj/robosuite) confirmed via WebFetch: the LIBERO fork does **not** carry the `skip_render_images` kwarg, so it's a hard either/or with cube_stack support — can't use a single fork for both.

The user already has a separate `packages/robotics/libero/` Ryzer for standalone (non-cap-x) LIBERO benchmark use; that covers the vanilla-libero use case.

### What would unblock this

1. Solve T1-B (SAM3 fallback / replacement) so a custom LIBERO YAML with PyRoKi-only `api_servers` can be authored against the cap-x LIBERO integration.
2. Or accept "oracle-mode-only LIBERO": ship a custom YAML that uses PyRoKi-only and hard-codes `use_oracle_code: true`. This proves the LIBERO sim stack works without ever exercising perception, and might be a useful smoke test, but it doesn't actually run any LIBERO research workflows.

### Sketch of the eventual port (when T1-B lands)

- Add a sibling `capx-libero` Ryzer (chained: `ryzers build capx capx-libero`) that:
  1. `pip uninstall robosuite` (the uynitsuj fork).
  2. `pip install --no-deps "robosuite @ git+https://github.com/Max-Fu/robosuite.git@maxf/egl_context"`.
  3. `pip install libero==0.1.1 easydict==1.9 robomimic==0.2.0 einops==0.4.1 thop==0.1.1-2209072238 bddl==1.0.1 future==0.18.2 cloudpickle==2.1.0 gym==0.25.2 scikit-learn`.
  4. Ship one or more `env_configs/libero/*_rocm.yaml` files that use only `launch_pyroki_server.main` in their `api_servers` block.
- Layered design lets users `ryzers build capx` (cube_stack) or `ryzers build capx capx-libero` (LIBERO) at choice; we don't try to support both stacks in one image.

---

## Tier 1.D — `--use-oracle-code` (no-LLM) eval mode  [DONE]

### Goal

Make it possible to validate the sim+IK pipeline without an OpenRouter / vLLM backend, since the smoke test only checks imports.

### Findings

- `franka_robosuite_cube_stack_privileged.yaml` resolves to `FrankaPickPlaceCodeEnv` (`capx/envs/tasks/franka/franka_pick_place.py`), which has `oracle_code = ORACLE_CODE` baked in (line 56). Eight Franka tasks ship with oracle code; r1pro_pickup_radio and r1pro_pickup_trash also do.
- `python3 capx/envs/launch.py --use-oracle-code True --total-trials 1 --num-workers 1 --record-video False --config-path .../franka_robosuite_cube_stack_privileged.yaml` ran clean, reward 1.0, task completed, **no LLM proxy required**, in 35-37s. `--server-url` is never reached when oracle is on, so an unset key just doesn't matter.
- Output goes under `<output_dir>/oracle/<task>/...` (capx prefixes with the model name; `oracle` is used when `use_oracle_code` is True).

### What we shipped

- `demo.sh` runs the privileged cube-stack oracle trial; `Dockerfile` copies it to `/ryzers/demo_capx.sh` and `chmod +x`. README documents `ryzers run /ryzers/demo_capx.sh`.
- This gives us a no-API-key smoke that exercises the full sim + PyRoKi loop, complementing the import-only smoke test.

### Future work

- Could extend `demo.sh` with a `--task` flag to run any of the eight oracle-equipped tasks (lift, nut_assembly, pick_place, spill_wipe, two_arm_lift, two_arm_handover, cube_restack, cube_stack-via-pick_place). Currently fixed to cube_stack.

---

## Tier 2.A — local LLM backend  [RESEARCHED, not built]

### Goal

Run cap-x evals fully offline (no OpenRouter), using a local LLM behind cap-x's standard OpenAI-compatible `--server-url` interface. Cap-x's `capx/envs/launch.py` accepts any URL via `--server-url`, so the integration boils down to "have a local OpenAI-compatible server listening somewhere".

### Path candidates we evaluated (2026-05-12)

#### A. AMD lemonade-sdk Ryzer (chained: `ryzers build capx lemonade-sdk`)

**Status:** broken upstream as of v10.4.0. The Dockerfile pulls `lemonade-server_10.0.0_amd64.deb` from GitHub releases; that filename was removed in v10.4.0 (only `.rpm` is published now). Verified: `wget` returns 404 in container.

**Fix path:** edit `packages/llm/lemonade-sdk/Dockerfile` to either pin to an older tag (last known-good is whichever release still shipped the .deb), pull the RPM and convert with `alien`, or switch to the PyPI package (`pip install lemonade-sdk` — installs cleanly but does **not** create a `lemonade-server` console script in 9.1.4; needs `python3 -m lemonade.server` or similar — verify before relying on it). This is upstream Ryzer maintenance, not part of capx.

#### B. llama.cpp Ryzer (chained: `ryzers build llamacpp capx`)

**Status:** the `packages/llm/llamacpp/` Ryzer builds llama.cpp from source with HIP support (`-DGGML_HIP=ON -DAMDGPU_TARGETS=$GFXSTRING`). Shipping `llama-server` exposes an OpenAI-compatible API on a configurable port. Should compose cleanly with capx as a layered Ryzer.

**Catch:** the llamacpp Dockerfile expects `HSA_OVERRIDE_GFX_VERSION` and `GFXSTRING` build_args (capx doesn't); when chaining you need a `config.yaml` somewhere setting them. Also a clean two-image deployment (`docker run -p 8000:8000 llamacpp:llama-server` + `docker run capx --server-url http://host.docker.internal:8000/v1/chat/completions`) avoids the build-arg coupling entirely.

**Sample workflow once wired up:**
```sh
# Container 1 (LLM backend)
ryzers build llamacpp
ryzers run bash -c "llama-server -hf <repo>/<model.gguf> --port 8000 --host 0.0.0.0"
# Container 2 (capx)
ryzers run --device=/dev/kfd ... bash -c "
    cd /ryzers/cap-x &&
    python3 capx/envs/launch.py \
        --config-path env_configs/cube_stack/franka_robosuite_cube_stack_privileged.yaml \
        --server-url http://localhost:8000/v1/chat/completions \
        --model llamacpp"
```

#### C. vLLM-ROCm baked into capx

**Status:** vanilla `pip install vllm` fetches a CUDA-built wheel (vllm 0.20.2 from PyPI). Verified the wheel imports but `vllm.LLM` fails because (a) the compiled `_C.so` kernels are CUDA, not HIP, and (b) it pulls many `nvidia-*` deps even with `--no-deps`.

The right path is `pip install` from AMD's ROCm vLLM index or building from source with `VLLM_TARGET_DEVICE=rocm`. Both add multi-GB to the image and have non-trivial dep conflicts with our pinned `numpy<2` / `transformers>=5,<6` / ROCm torch — sandbox-time-to-validate is hours, not minutes. Punt unless we get a concrete user demand for it.

#### D. ollama (separate container)

**Status:** unpursued in this session. `curl https://ollama.com/install.sh | sh` is the standard. Could be a 5-minute add as a separate Ryzer. Lacks tensor parallelism but fine for cap-x's single-call eval pattern.

### Recommendation

For a near-term local-LLM story: **option B (chained llamacpp + capx)** is the cheapest path. Track upstream lemonade-sdk Ryzer fix; once that lands, switch the docs to recommend it (better Ryzen-AI integration than llama.cpp). Skip C until someone needs vLLM-specific features.

### What needs to change in capx (none, today)

`capx/envs/launch.py` already takes `--server-url` and `--model`; the upstream OpenRouter proxy under `capx/serving/openrouter_server.py` is just a (configurable) reverse-proxy that we don't need when the local backend is already OpenAI-compatible. The HF transformers Sam3+Sam2 shim from T1-B doesn't depend on any LLM backend. So once a local LLM server exists (any of A/B/C/D), capx just needs the right URL passed in — no source patches required.

---

## Tier 2.B — flash-attn ROCm  [DEFERRED]

### Goal

`from flash_attn import flash_attn_func` working on ROCm. Required only by `molmo` (T3-B) and `verl`-based RL training (T3-C). Not on the critical path for any current cap-x workload.

### Path candidates (sketched, not validated)

1. **AOTriton flash-attn.** AMD's tritonized flash-attn (`https://github.com/ROCm/aotriton`). PyTorch 2.5+ has `torch.nn.functional.scaled_dot_product_attention` ROCm fast path that uses AOTriton under the hood. Some libraries can be coaxed into using SDPA instead of `flash_attn` directly, eliminating the dep. Cap-x doesn't import flash-attn anywhere in `capx/` — it only matters if the LLM backend (vLLM/verl) requires it.
2. **`composable_kernel` flash-attn.** `pip install flash-attn --no-build-isolation` against ROCm PyTorch sometimes works (recent flash-attn upstream has ROCm support for gfx94x/gfx95x; gfx11xx Ryzen AI is less well covered).
3. **`xformers` ROCm.** Memory-efficient attention; nothing in capx uses it but it's the typical fallback for LLM trainers.

### Why we're deferring

Without a local LLM backend (T2-A) baked in, flash-attn is dead weight. If a user wires up a vLLM-ROCm or verl-ROCm pipeline later, that's the right time to revisit this — they'll know which API surface needs to be satisfied.

---

## Tier 3 — hard / risky / deferred

We have not started any of these. Quick sketches so the next agent doesn't repeat investigation:

### T3-A — contact_graspnet_pytorch (CUDA → ROCm)

Used by every default cube_stack and LIBERO config to generate grasp candidates from depth/pointclouds. Custom CUDA kernels (PointNet++ ops, KNN) — direct ROCm port is nontrivial (~days). Two pragmatic alternatives:
1. **Replace with a heuristic.** Cap-x's grasp candidates are a list of `(pos, quat)` poses ranked by score. For top-down tabletop grasps you can synthesize a serviceable list from the SAM3 mask centroid + a fixed downward orientation. This unblocks the LIBERO and default cube_stack configs without porting any GPU kernels.
2. **Find an existing PyTorch-only fork.** Some `contact_graspnet_pytorch` forks have CPU paths or HIP ports — search github before you start writing kernels.

This is the **single biggest unlock** remaining for ROCm cap-x — would unblock T1-C (LIBERO) and let the default `franka_robosuite_cube_stack.yaml` work without `_privileged`.

### T3-B — molmo extra

The `molmo` extra in pyproject installs `transformers + tensorflow + accelerate + bitsandbytes` for a Molmo-based image grounding model. Tensorflow has ROCm support, but cap-x pins specific tensorflow versions that may not match. Lower priority; cap-x can use the SAM3 server's text-prompt path for the same purpose.

### T3-C — verl-based RL training

Depends on T2-A (vLLM-ROCm) and T2-B (flash-attn-ROCm) being solid. Not blocking any current capx user.

### T3-D — full SAM3 with HIP-ported CUDA kernels

We picked the HF transformers fallback (T1-B) instead. The Max-Fu/sam3 kernels could be HIP-ported but the HF-transformers Sam3 has the same model weights and is functionally equivalent for cap-x's use case. **Don't pursue this.**

---

## What's next — concrete handoff

If picking this up cold, here's the highest-leverage next step:

1. **Read `Dockerfile`, `test.sh`, `demo.sh`, `demo_sam3.sh`, `launch_sam3_server.py`, this file.** That's the whole engineering surface. Less than 1k LOC total.
2. **Try T3-A heuristic-grasp approach.** This is the highest-value remaining unlock — a small `launch_contact_graspnet_server.py` overlay (analogous to our SAM3 shim) that uses SAM3 mask centroid + a top-down quat as a single grasp candidate. Should be ~150 LOC. If it works, it unblocks the entire default-config cube_stack flow on ROCm and most LIBERO configs (modulo the robosuite-fork conflict).
3. **Or try T2-A option B.** Layer llamacpp + capx as a chained Ryzer; verify a small Qwen GGUF model serves correctly and capx eval drives end-to-end with no API key.

Each of those is a self-contained 1-2 hour session; pick one based on whether the next user prefers "more configs work" (T3-A) or "no API key needed" (T2-A).

---

## Operational notes

### Build / test loop

```sh
# rebuild (~2-3 min once layers are warm)
ryzers build capx

# fast smoke test (3-5 sec, no GPU traffic)
docker run --rm --shm-size 16G --cap-add=SYS_PTRACE --network=host --ipc=host \
    -e HSA_OVERRIDE_GFX_VERSION=11.0.0 -e MUJOCO_GL=egl -e PYOPENGL_PLATFORM=egl \
    --device=/dev/kfd --device=/dev/dri --security-opt seccomp=unconfined \
    --group-add video --group-add render --shm-size=2g \
    ryzerdocker

# end-to-end privileged eval (needs OPENROUTER key on host at /tmp/openrouterkey)
# see commit history / chat for the runner script template; deletes key after run
```

### Git hygiene for this porting work

- Branch is `0506_capx`. Do **not** push without explicit user approval.
- Commit per tier item (T1-A, T1-B, ...) so each can be reverted independently.
- Don't stage `ryzers/__init__.py` (the base-image bump is a separate concern owned by the user).
- Don't stage `workspace/`, `output/`, `.claude/`, `ryzers.run.*.sh`, `ryzers/_ryzers.yaml`, or any `.deb` files — those are local artifacts.

### Cost / time discipline for end-to-end evals

- Always start with `--total-trials 1 --num-workers 1` and `openai/gpt-4o-mini` (cheap, capable enough for cube_stack).
- The PyRoKi cold start downloads `example-robot-data` (~30s on first run); subsequent runs are warm.
- A 1-trial privileged cube-stack run takes ~45s wall time end-to-end and a few cents of OpenRouter spend.

### Don't lose track of these files when iterating

- `Dockerfile` — install steps; the bulk of porting work lives here
- `test.sh` — install validator; assertions for fork presence and numpy 1.x
- `README.md` — user-facing docs (what works, what doesn't, how to run)
- `ROCM_PORTING.md` — this file (engineering notes / plan / findings)
