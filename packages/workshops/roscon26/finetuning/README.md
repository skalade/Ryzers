# ROSCon 2026: Fine-tuning on GPUs: from cloud to edge

Standalone [MolmoAct2](https://huggingface.co/collections/allenai/molmoact) ROCm
environment, flattened from the Ryzers `molmoact2` package with JupyterLab added.

## Build

```
docker build -t roscon26_finetuning -f Dockerfile .
```

## Run

Starts JupyterLab on port `8888`. Set `HF_TOKEN` for faster/gated downloads.

```
docker run -it --rm \
  --shm-size 16G \
  --cap-add=SYS_PTRACE \
  --security-opt seccomp=unconfined \
  --network=host \
  --ipc=host \
  -e HF_HOME=/root/.cache/huggingface \
  -e HF_TOKEN=${HF_TOKEN:-} \
  -v $PWD/notebooks:/ryzers/notebooks \
  -v $PWD/workspace/.cache/huggingface:/root/.cache/huggingface \
  --device=/dev/kfd \
  --device=/dev/dri \
  --group-add video \
  --group-add render \
  roscon26_finetuning
```

## Test

```
./run.sh /ryzers/test_torch.sh
./run.sh /ryzers/test_molmoact2.sh
```

Expected output:

```
Testing ROCm torch environment...
torch            : 2.10.0+rocm7.2.2.git23d69b29
torch.version.hip: 7.2.53211
device[0]        : Radeon 8060S Graphics
matmul ok        : sum=-4704.191
PASS: ROCm torch env OK
```

```
Testing MolmoAct2 dependencies...
transformers     : 4.57.6
accelerate       : 1.14.0
huggingface_hub  : 0.36.2
deps import ok   : einops, fastapi, json_numpy, safetensors, sentencepiece
PASS: MolmoAct2 deps OK
```
