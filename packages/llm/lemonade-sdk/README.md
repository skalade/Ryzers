# Lemonade SDK Docker Setup

Lemonade is a local AI application platform that serves optimized LLMs right from your own GPUs and NPUs. It provides an OpenAI-compatible API at `http://localhost:8000/api/v1` and supports GGUF, ONNX, SafeTensors, and FLM model formats.

## Build & Run (GPU)

By default, the package builds with GPU support only:

```sh
ryzers build lemonade-sdk
ryzers run
```

## Build & Run (NPU)

To enable NPU support via [FastFlowLM](https://github.com/FastFlowLM/FastFlowLM) (FLM), set the `ENABLE_NPU` build argument in `config.yaml`:

```yaml
build_arguments:
- "ENABLE_NPU=true"
```

This installs the XRT/XDNA runtime and FLM backend. The XDNA kernel driver must also be installed on the host — see the [xdna package README](../../npu/xdna/README.md) for setup instructions.

Also uncomment the NPU run flags in `config.yaml`:

```yaml
docker_extra_run_flags:
- "--device=/dev/accel/accel0:/dev/accel/accel0"
- "--ulimit memlock=-1"
```

Then build and run:

```sh
ryzers build lemonade-sdk
ryzers run bash
```

Inside the container, use FLM directly:

```sh
flm list                    # List available NPU models
flm pull llama3.2:1b        # Download a model
flm run llama3.2:1b         # Run interactively on NPU
flm serve llama3.2:1b       # Start an OpenAI-compatible server on NPU
```

Or use the lemonade SDK interface:

```sh
lemonade -i llama3.2 flm-load llm-prompt     # Run a model on NPU
lemonade -i llama3.2 flm-load flm-bench      # Benchmark a model on NPU
```

## References

- [Lemonade Server](https://lemonade-server.ai/)
- [GitHub - lemonade-sdk/lemonade](https://github.com/lemonade-sdk/lemonade)
- [PyPI - lemonade-sdk](https://pypi.org/project/lemonade-sdk/)
- [FastFlowLM](https://github.com/FastFlowLM/FastFlowLM)

Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
