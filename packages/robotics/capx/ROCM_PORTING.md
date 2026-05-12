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

## The seven gotchas we have hit so far

These are the bugs that cost meaningful time. The smoke test now defends against #2, #3, #4 and the build itself defends against #5.

| # | Gotcha                                                              | Symptom                                                                 | Fix                                                          |
|---|---------------------------------------------------------------------|-------------------------------------------------------------------------|--------------------------------------------------------------|
| 1 | `uv sync --extra robosuite` builds sam3 CUDA extensions             | `OSError: CUDA_HOME environment variable is not set`                    | Don't use uv at all; pip install + `-e cap-x --no-deps`      |
| 2 | `PyOpenGL-accelerate==3.1.6` Cython-precompiled for old CPython     | `error: 'PyLongObject' has no member named 'ob_digit'` (Py 3.12)        | Drop both `PyOpenGL-accelerate` pin and `PyOpenGL==3.1.6`    |
| 3 | Older PyOpenGL lacks EGL device extensions on Py 3.12               | `AttributeError: module 'OpenGL.EGL' has no attribute 'EGLDeviceEXT'`   | Pin `PyOpenGL>=3.1.7`                                        |
| 4 | Vanilla robosuite from PyPI                                         | `TypeError: MujocoEnv.step() got unexpected kwarg 'skip_render_images'` | Install uynitsuj/robosuite fork                              |
| 5 | Robosuite fork pulls `mink>=1.0`, which forces numpy 2.x            | `numpy must stay on 1.x for robosuite/mink compatibility, got 2.4.4`    | Install fork with `--no-deps`; add `numba`, `termcolor`      |
| 6 | JAX `>=0.4.30` requires numpy 2.x; pyroki has no upper bound        | numpy gets force-upgraded                                               | Pin `jax<0.4.30, jaxlib<0.4.30` (matches upstream override)  |
| 7 | Default config tries to start three perception servers              | `ModuleNotFoundError: No module named 'sam3' / pyroki / ...`            | Use a `_privileged.yaml` config; only PyRoKi is shipped here |

---

## Porting tiers

These are the units of work for the rest of this document. Each entry is sized so a single agent session can finish it. **We're working through them in order.** Status is updated as we go.

### Tier 1 — low-hanging fruit (single dep, mechanical change)

| ID    | Item                                                | Status      | Pointer                                              |
|-------|-----------------------------------------------------|-------------|------------------------------------------------------|
| T1-A  | `jax[rocm]` for pyroki (faster IK)                  | not started | see "Tier 1.A" below                                 |
| T1-B  | SAM3 ROCm/CPU fallback investigation                | not started | see "Tier 1.B" below                                 |
| T1-C  | LIBERO-PRO support                                  | not started | see "Tier 1.C" below                                 |
| T1-D  | `--use-oracle-code` (no-LLM) eval mode              | not started | see "Tier 1.D" below                                 |

### Tier 2 — medium effort (multi-dep / build-from-source)

| ID    | Item                                                | Status      | Pointer                                              |
|-------|-----------------------------------------------------|-------------|------------------------------------------------------|
| T2-A  | vLLM-ROCm as local LLM backend                      | not started | see "Tier 2.A" below                                 |
| T2-B  | flash-attn ROCm (Triton / Composable Kernel)        | not started | see "Tier 2.B" below                                 |

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

## Tier 1.A — jax[rocm] for pyroki

### Goal

Replace CPU JAX with ROCm JAX so PyRoKi's IK/collision constraint solving runs on the iGPU. PyRoKi is JAX-only (no PyTorch fallback), and JAX kernels are jit-compiled, so this could noticeably speed up multi-trial evals.

### Constraints

- Need to keep `jax<0.4.30` so numpy stays 1.x (gotcha #6 above).
- Need a `jaxlib` ROCm wheel that matches the same minor version.
- ROCm version on the host must line up with whatever AMD's prebuilt jaxlib expects.

### Open questions

1. Does AMD ship a `jax[rocm]==0.4.x` wheel for ROCm 7.2? Their official channel is `https://github.com/ROCm/jax/releases` and the `pip install jax-rocm60-pjrt jax-rocm60-plugin` flow.
2. Does pyroki actually exercise enough JAX to justify a GPU; what's the speedup on a Franka 7-DOF chain with 41 collision spheres?
3. Will an ROCm jaxlib install conflict with anything else (e.g. by pulling `numpy>=2`)?

### Plan

1. Spawn a fresh `ryzerdocker` shell, try `pip install jax-rocm60-plugin jax-rocm60-pjrt` (matching AMD's docs) and see what versions resolve.
2. If it imports and `jax.devices()` shows ROCm, run a synthetic 100-call IK benchmark with both CPU and ROCm backends to measure the real win.
3. If wins are real, bake into Dockerfile under a build arg `JAX_BACKEND=rocm|cpu` so users can pick.
4. If versions force numpy 2.x or break, rollback and document under "won't fix".

### Findings

(populate as we go)

---

## Tier 1.B — SAM3 ROCm fallback

### Goal

See if SAM3 (or a ROCm-friendly equivalent) can be made to run as the visual-grounding perception server, unlocking the default cube-stack config and other SAM3-using YAMLs.

### What SAM3 actually is

[SAM3](https://github.com/facebookresearch/sam3) is Meta's "Segment Anything 3" — a point/box/text-prompted segmentation model. cap-x uses Max-Fu's fork (`https://github.com/Max-Fu/sam3`) which adds a few cap-x-specific entry points. The CUDA build is in the SAM3 wrapper around the model's transformer; the underlying ViT itself is plain PyTorch.

### Hypotheses to test

1. **PyTorch-only SAM3 path.** Maybe the CUDA C++ extensions are an *optimization*, not a requirement. Many SAM-family repos build a CUDA-fast attention kernel but fall back to plain torch SDPA on import failure. If that's true, we can install Max-Fu/sam3 with `setup.py` flags to skip the CUDA build (e.g. `SAM3_NO_CUDA=1`) and still get a working server, just slower.
2. **Drop-in alternative.** If (1) doesn't pan out, swap `launch_sam3_server.py` for a tiny FastAPI shim that calls vanilla SAM2 / mobile-SAM (already a Ryzer in `packages/vision/mobilesam`) over the same `/segment` endpoint signature. This is a cap-x patch, not a SAM3 port.
3. **OpenCLIP + bbox grounding.** Some configs only need text-prompted segmentation; CLIP-based methods on ROCm work today (see `packages/vision/dinov3`).

### Plan

1. Read `Max-Fu/sam3/setup.py` and `capx/serving/launch_sam3_server.py` to find the import path. Specifically look for `try: import _sam3_cuda except: ...` patterns.
2. Try building Max-Fu/sam3 with `--no-build-isolation` and a flag that forces CPU. Expected to fail.
3. If (2) fails, prototype the FastAPI shim approach. It only needs to handle whatever endpoints `capx.envs.tasks.<task>.get_object_pose` calls in the privileged-vs-non-privileged path.

### Findings

(populate as we go)

---

## Tier 1.C — LIBERO-PRO support

### Goal

Run LIBERO benchmark tasks (130 task suites) on this image. LIBERO is plain Python + a robosuite fork; should be much easier than SAM3.

### Constraints

- Upstream cap-x docs say LIBERO needs a *separate* Python 3.12 venv because its `robosuite==1.4.0` fork conflicts with the 1.5.x fork we use for the Robosuite envs.
- Both forks live in `[tool.uv.sources]` under different extras, but pip can only have one `robosuite` installed at a time.

### Hypotheses to test

1. **One-image-two-venvs.** Add a sibling venv at `/opt/venv-libero` containing only the LIBERO stack. Switching is a `source` away. Big image but conceptually simple.
2. **Single venv with the LIBERO fork.** Use Max-Fu/robosuite (`maxf/egl_context` branch) for *both* LIBERO and the default Robosuite envs. Risk: the cap-x cube_stack tasks may rely on uynitsuj-fork-specific behavior that maxf doesn't have.
3. **Branch the Ryzer.** Ship a separate `capx-libero` Ryzer that builds on the existing `capx` and replaces the robosuite install with the LIBERO fork. Lets users pick at build time.

### Plan

1. Try (2) first — install Max-Fu/robosuite + libero==0.1.1 + bddl into a sandbox container. If both `import robosuite` and the cube_stack eval still pass, this is the cleanest answer.
2. If (2) breaks the cube_stack eval, fall back to (1) or (3).

### Findings

(populate as we go)

---

## Tier 1.D — `--use-oracle-code` (no-LLM) eval mode

### Goal

Make it possible to validate the sim+IK pipeline without an OpenRouter / vLLM backend, since the current smoke test only checks imports.

### Why it's easy

`capx/envs/launch.py` already accepts `--use-oracle-code True` which bypasses the LLM and runs pre-defined oracle code. We just need to verify it runs to completion on this image and document the invocation.

### Plan

1. Check `env_configs/cube_stack/hillclimb/` and any `oracle_code` blocks in YAML configs to find a config with `use_oracle_code: true` baked in (or know that `--use-oracle-code True` works on the privileged config).
2. Add an optional `demo.sh` (and `config.yaml` `docker_extra_run_flags` mention) that runs an oracle eval as a "no API key needed" smoke check.

### Findings

(populate as we go)

---

## Tier 2.A — vLLM-ROCm

### Goal

Replace OpenRouter with a local vLLM-ROCm server so users can run cap-x evals fully offline.

### Plan sketch

- Use AMD's `rocm/vllm` image as a sibling Ryzer or a build-time install in a separate stage.
- Wire it into `capx/serving/openrouter_server.py` style by swapping `--server-url` to point at the local vLLM endpoint.

This is medium effort because vLLM-ROCm has been stable for a few releases but pinning a torch + vllm + xformers triple that all share the same ROCm and Python versions is fiddly.

---

## Tier 2.B — flash-attn ROCm

### Goal

Get a `skip_render_images`-style green-light on `from flash_attn import flash_attn_func` running on ROCm. Required by molmo and verl.

### Path candidates

1. **AOTriton flash-attn.** AMD's tritonized flash-attn (`https://github.com/ROCm/aotriton`). PyTorch 2.5+ has `torch.nn.functional.scaled_dot_product_attention` ROCm fast path that uses it under the hood; some libraries can be coaxed into using SDPA instead of `flash_attn`.
2. **`composable_kernel` flash-attn.** `pip install flash-attn --no-build-isolation` against ROCm-built PyTorch sometimes works directly.
3. **`xformers` ROCm.** Has its own memory-efficient attention; cap-x doesn't currently use it but verl-style trainers can be told to.

---

## Tier 3 (placeholder)

We're not there yet; will fill in if/when Tier 2 is solid.

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
