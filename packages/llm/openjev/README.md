# OpenJev Docker Setup

[OpenJev](https://huggingface.co/openjev/openjev) is an open-weights decision model. You describe the decision in the request, with your own labels, and it returns a choice, a yes/no probability, or a score, with no text to parse.

This package serves it the way the model card was measured: vLLM 0.29.0 with online FP8, and the upstream helper in front. The helper reads option-letter scores from one forward pass. The base image is `vllm/vllm-openai-rocm:v0.29.0`, whose PyTorch build includes `gfx1151` (Ryzen AI MAX / AI 300).

Weights are not in the image. The bfloat16 checkpoint is about 54 GB and is downloaded on the first serve into `workspace/openjev/model`.

## Build & Run

```sh
ryzers build openjev
ryzers run
```

`ryzers run` checks vLLM, the pinned helper libraries, and the helper file. It does not download or load the weights.

Start the decision server:

```sh
ryzers run /ryzers/serve_openjev.sh
```

Ryzers runs with host networking, so the helper is at `http://127.0.0.1:3000`. vLLM stays on `127.0.0.1:8000`.

```sh
curl -s http://127.0.0.1:3000/v1/systemone \
  -H 'Content-Type: application/json' \
  -d '{
    "model": "openjev",
    "state": "Customer message: I was charged twice for my order last week and nobody has replied.",
    "questions": {
      "route": {"type": "choice", "instructions": "Which team should handle this?",
                "criteria": {"billing": null, "shipping": null, "technical": null}},
      "angry": {"type": "noul", "instructions": "Is the customer angry?"},
      "urgency": {"type": "score", "instructions": "How urgent is this?",
                  "criteria": ["can wait", "this week", "today", "right now"]}
    }
  }'
```

## Volumes

- `workspace/openjev/model`: model weights, reused across runs
- `workspace/.cache/huggingface`: Hugging Face cache

## Memory

Online FP8 still starts from the 54 GB bfloat16 checkpoint. A Ryzen AI MAX with 128 GB of unified memory is the machine this is aimed at. `OPENJEV_GPU_MEMORY_UTILIZATION` defaults to `0.90`, the measured setting. Lower it if the iGPU's share of system RAM is smaller than that.

The vLLM log inside the container is `/tmp/openjev-vllm.log`.

## Exposing the helper

The helper binds `127.0.0.1`. To listen on another address, set `OPENJEV_SHIM_HOST` and `SHIM_TOKEN`. The script refuses a non-loopback bind when `SHIM_TOKEN` is empty. vLLM stays on loopback. The helper speaks plain HTTP.

## Licence

OpenJev weights are [CC BY-NC 4.0](https://huggingface.co/openjev/openjev). The files in upstream `helper/` and `serve/` are Apache 2.0. This package's scripts are MIT.

## References

- https://huggingface.co/openjev/openjev
- https://huggingface.co/openjev/openjev/blob/main/serve/SERVE.md
- https://github.com/vllm-project/vllm/releases/tag/v0.29.0

Copyright(C) 2026 Advanced Micro Devices, Inc. All rights reserved.
