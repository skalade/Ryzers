### openpi

This directory contains the docker configuration files to run [openpi from Physical Intelligence](https://github.com/Physical-Intelligence/openpi).

### Build and run the Docker Image

```sh
ryzers build openai
#ryzers run
docker run -it --rm --shm-size 32G --privileged --cap-add=SYS_PTRACE  --network=host --ipc=host -e HSA_OVERRIDE_GFX_VERSION=11.0.0 --device=/dev/kfd --device=/dev/dri --security-opt seccomp=unconfined --group-add video -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ryzerdocker 
```

### Install openpi
```sh
GIT_LFS_SKIP_SMUDGE=1 uv sync
GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .
```

### Download model
```sh
uv run scripts/serve_policy.py --download-only policy:checkpoint --policy.config=pi0_droid --policy.dir=gs://openpi-assets/checkpoints/pi0_droid
```

### Apply PyTorch support patch
```sh
cp -r ./src/openpi/models_pytorch/transformers_replace/* .venv/lib/python3.11/site-packages/transformers/
```

### Convert JAX models to PyTorch
```sh
uv run examples/convert_jax_model_to_pytorch.py \
    --checkpoint_dir /root/.cache/openpi/openpi-assets/checkpoints/pi0_droid \
    --config_name pi0_droid \
    --output_path /root/.cache/openpi/openpi-assets/checkpoints/pi0_droid
```

### Run the openpi inference test
```sh
uv run test_pi0_pytorch.py
```

### How to debug page fault on GPU
Run 'dmesg' on the host in a separate terminal.
```sh
dmesg -wH | grep -i 'amdgpu\|gpu\|fault'
```
Run python's unbuffered mode with uv, so that we can keep the log file.
```sh
script -c "uv run python -u test_pi0_pytorch.py --debug" debug_test_pi0_pytorch.log
```

If want to clear cache and debug, use
```sh
uv run python -u test_pi0_pytorch.py --debug --clear-cache
```

Copyright(C) 2025 Advanced Micro Devices, Inc. All rights reserved.