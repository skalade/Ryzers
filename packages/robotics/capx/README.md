# CaP-X Docker Setup

[CaP-X](https://github.com/capgym/cap-x) is an open-access framework for systematically studying **Code-as-Policy** agents in robot manipulation. It bundles four components:

| Component      | What it does                                                                                                                                              |
| -------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **CaP-Gym**    | Interactive Gymnasium environments where agents control robots by generating Python code that composes perception and control primitives.                |
| **CaP-Bench**  | Systematic benchmark across abstraction levels, interaction modes, and visual grounding modalities (8 tiers, S1-S4 single-turn, M1-M4 multi-turn).       |
| **CaP-Agent0** | Training-free agentic framework with multi-turn visual differencing, auto-synthesized skill libraries, and parallel ensembled reasoning.                 |
| **CaP-RL**     | Reinforcement learning on the coding agent via GRPO/VeRL, post-training language models with environment rewards.                                        |

## Build & Run

```sh
ryzers build capx
ryzers run                                  # smoke test (imports only, ~5s, no GPU traffic)
ryzers run /ryzers/demo_capx.sh             # oracle-code cube-stack demo (~35s, no API key)
ryzers run /ryzers/demo_sam3_capx.sh        # SAM3-on-ROCm shim smoke (~30s, no HF auth needed)
ryzers run /ryzers/bench_capx.sh            # multi-task benchmark, 7 tasks x 3 trials (~3m, no API key)
```

`ryzers run` executes a smoke test that imports `capx`, `torch`, `robosuite`, `mujoco`, `gymnasium`, `transformers`, `ray`, `pyroki`, and asserts the uynitsuj-fork-only `skip_render_images` kwarg and numpy 1.x.

`ryzers run /ryzers/demo_capx.sh` runs a **single oracle-code trial** of the Robosuite Franka cube-stack task end-to-end (PyRoKi server + sim + privileged controller), with no LLM proxy or API key required. Expect reward 1.0 / task completed in ~35s on a warm cache.

`ryzers run /ryzers/demo_sam3_capx.sh` starts the SAM3-on-ROCm shim and runs a `/segment_point` round-trip against synthetic data using ungated SAM2 weights. Set `HF_TOKEN=...` to also exercise the gated SAM3 `/segment` text-prompt endpoint.

`ryzers run /ryzers/bench_capx.sh` runs a real multi-task CaP-X benchmark on the seven Robosuite tasks whose privileged configs are PyRoKi-only (cube_stack, cube_lifting, cube_restack, nut_assembly, spill_wipe, two_arm_lift, two_arm_handover). Defaults to 3 trials/task, ~3 min wall time, no API key required. Override with `TRIALS=N` or `TASKS=cube_stack,nut_assembly`.

### Real benchmark numbers we got

Captured on 2026-05-12 from a single `bench_capx.sh` run on a Ryzen AI iGPU
with `HSA_OVERRIDE_GFX_VERSION=11.0.0`, oracle-code, no LLM:

| task              | success | wall      | notes                                |
|-------------------|---------|-----------|--------------------------------------|
| cube_stack        | 5/5     | 39.5s     | 5-trial run                          |
| cube_lifting      | 3/3     | 10.8s     | simplest task                        |
| nut_assembly      | 2/3     | 16.1s     | one trial got reward 0.0004          |
| spill_wipe        | 3/3     | 14.3s     |                                      |
| two_arm_handover  | 1/3     | 59.3s     | hardest task; oracle is partial      |
| **total**         | **14/17** | **2m 41s** |                                    |

These match the spread cap-x's authors report upstream — simple top-down manipulations are reliable, two-arm coordination and threaded nut assembly are not, oracle code is not deterministic across initial seeds. Use this as the no-API-key baseline before any LLM run.

## ROCm vs upstream CUDA

CaP-X targets CUDA-only NVIDIA GPUs upstream and installs via `uv sync`, which would pull in several CUDA-bound build-from-source dependencies. This Ryzer takes a **ROCm-native** approach instead:

- **Base image:** `rocm/pytorch` (Python 3.12, PyTorch 2.10 with ROCm 7.2). The base image's PyTorch is preserved — there is no separate uv venv that would fetch a CUDA wheel from PyPI.
- **Install method:** plain `pip3` for the runtime deps that have wheels (`gymnasium`, `mujoco`, `robosuite`, `transformers>=5`, `ray`, `fastapi`, `viser`, `trimesh`, `open3d`, `pyrender`, `pyroki`, etc.) and an editable `pip install --no-deps -e cap-x` so `import capx` works against the cloned source tree. The pip install is split into staged transactions to keep pip's resolver from replacing the ROCm torch with a vanilla CUDA torch from PyPI.
- **PyRoKi (IK perception server) is installed** — pure-Python/JAX, runs on CPU JAX. We pin `jax<0.4.30, jaxlib<0.4.30` to keep numpy on 1.x for robosuite/mink compatibility (matches upstream's `[tool.uv]` overrides).
- **SAM3 perception server is replaced** with a HuggingFace-transformers-based shim (`packages/robotics/capx/launch_sam3_server.py`) that overlays `capx/serving/launch_sam3_server.py` at build time. The shim exposes the same `/segment` and `/segment_point` endpoints on port 8114, but routes to `transformers.Sam3Model` (text-prompt) and `transformers.Sam2Model` (point-prompt) — both pure PyTorch, ROCm-compatible, no CUDA C++ extensions. SAM3 weights (`facebook/sam3`) are HF-gated and require `huggingface-cli login` (or `HF_TOKEN`) to download; SAM2 weights are not gated.
- **Skipped on purpose** (still CUDA-only or NVIDIA-only):
  - `nvidia-curobo`, `contact_graspnet_pytorch`, `flash-attn`, `vllm`, `verl`, the `molmo` extra
  - The `b1k`/Isaac-Sim BEHAVIOR backend
  - The `LIBERO-PRO` extra (needs its own Python 3.12 venv with a pinned older robosuite fork that conflicts with 1.5.x)
- **Submodules are not cloned** — `git clone` runs without `--recurse-submodules` since every submodule is one of the skipped CUDA/NVIDIA components.

What you can run on this image:

- The CaP-X coding-agent harness (`capx/envs/launch.py`) against any config whose `api_servers` block uses only `capx.serving.launch_pyroki_server.main` and/or `capx.serving.launch_sam3_server.main` (with HF auth for SAM3 weights). That includes:
  - **All seven Robosuite `*_privileged.yaml` configs** (PyRoKi only) — cube_stack, cube_lifting, cube_restack, nut_assembly, spill_wipe, two_arm_lift, two_arm_handover. Confirmed working end-to-end with oracle code; works with any OpenAI-compatible LLM via `--server-url`.
  - **All eight `human_oracle_code/*_privileged_oracle.yaml` configs** (oracle-code variants of those tasks plus one LIBERO task).
  - The default `franka_robosuite_cube_stack.yaml` and similar SAM3-using configs (PyRoKi + SAM3) — the SAM3-on-ROCm shim from `launch_sam3_server.py` covers it; you only need `huggingface-cli login` for the gated SAM3 weights.
- Robosuite single-turn benchmarks via the `bench_capx.sh` runner.
- The interactive Web UI on port 8200.
- The OpenRouter / vLLM LLM proxies under `capx/serving/`.
- The SAM3-on-ROCm perception shim on port 8114 (text-prompt + point-prompt segmentation, pure PyTorch).

What will *not* work on this image:

- Any config whose `api_servers` references `launch_contact_graspnet_server` (e.g. the multi-turn VDM `*_reduced_api*.yaml` variants). Contact-GraspNet has CUDA C++ extensions and is not ported.
- LIBERO-PRO and BEHAVIOR task suites — see `ROCM_PORTING.md` Tier 1.C and 3 for status.
- The `verl`-based RL training pipeline (`docs/rl-training.md`) — depends on `vllm` and `flash-attn` CUDA wheels.

To check whether a config is ROCm-compatible:

```sh
grep -A1 _target_ env_configs/<task>/<config>.yaml
# `_target_` should list only: capx.serving.launch_pyroki_server.main
#                              and/or capx.serving.launch_sam3_server.main
# Anything mentioning `launch_contact_graspnet_server` will not work today.
```

## Running an Evaluation

The harness needs an OpenAI-compatible LLM proxy. Inside the container:

```sh
ryzers run bash

cd /ryzers/cap-x

# 1. Start an LLM proxy (OpenRouter shown; see upstream docs for vLLM/custom)
echo "sk-or-v1-your-key-here" > .openrouterkey
python3 capx/serving/openrouter_server.py --key-file .openrouterkey --port 8110 &

# 2. Run a Robosuite privileged single-turn benchmark
#    (privileged configs use only the PyRoKi perception server, so they
#    work on ROCm; the default config requires SAM3 + Contact-GraspNet,
#    which are CUDA-only and intentionally not installed.)
python3 capx/envs/launch.py \
    --config-path env_configs/cube_stack/franka_robosuite_cube_stack_privileged.yaml \
    --model "google/gemini-3.1-pro-preview"

# 3. (Optional) launch the interactive Web UI on port 8200
python3 capx/envs/launch.py \
    --config-path env_configs/cube_stack/franka_robosuite_cube_stack_privileged.yaml \
    --web-ui True
# Open http://localhost:8200
```

> Note: the upstream README uses `uv run --no-sync --active` everywhere. We've installed CaP-X into the system Python, so `python3` works directly — no uv invocation needed.

Ports `8110` (LLM proxy) and `8200` (Web UI) are exposed by this Ryzer's `config.yaml`. `HSA_OVERRIDE_GFX_VERSION=11.0.0` is set by default for Ryzen AI iGPU compatibility.

## References

- [CaP-X GitHub](https://github.com/capgym/cap-x)
- [Project Page](https://capgym.github.io/)
- [Configuration guide](https://github.com/capgym/cap-x/blob/main/docs/configuration.md)
- [Adding environments](https://github.com/capgym/cap-x/blob/main/docs/adding-environments.md)

Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
