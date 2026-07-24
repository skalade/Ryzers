# ROSCon 2026: Local Inference of embodied AI

Standalone image flattening the Ryzers `ros o3de rai lemonade-sdk` chain, with
JupyterLab added. Runs [RAI](https://github.com/RobotecAI/rai) robot agents
against models served locally by [Lemonade](https://lemonade-server.ai/).

## Build

```
docker build -t roscon26_local_inference --build-arg ROS_DISTRO=jazzy -f Dockerfile .
```

## Run

Starts JupyterLab on port `8888`; the Lemonade API is on `13305`.

```
docker run -it --rm \
  --shm-size 16G \
  --cap-add=SYS_PTRACE \
  --security-opt seccomp=unconfined \
  --network=host \
  --ipc=host \
  -e HSA_OVERRIDE_GFX_VERSION=11.0.0 \
  -v $PWD/notebooks:/ryzers/notebooks \
  -v $PWD/workspace/.cache/huggingface:/root/.cache/huggingface \
  -v $PWD/workspace/.cache/lemonade:/root/.cache/lemonade \
  --device=/dev/kfd \
  --device=/dev/dri \
  --group-add video \
  --group-add render \
  roscon26_local_inference
```

## Test

```
docker run --rm roscon26_local_inference /ryzers/test_ros.sh
docker run --rm roscon26_local_inference /ryzers/test_o3de.sh
docker run --rm roscon26_local_inference /ryzers/test_rai.sh
./run.sh /ryzers/test_lemonade-sdk.sh
```

The Lemonade test needs the iGPU and downloads a small model, so run it with the
device flags (via `run.sh`).
