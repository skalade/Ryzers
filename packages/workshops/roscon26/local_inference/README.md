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

Run each individual ryzer test (via `run.sh` so the iGPU device flags are set):

```
./run.sh /ryzers/test_ros.sh
./run.sh /ryzers/test_o3de.sh
./run.sh /ryzers/test_rai.sh
./run.sh /ryzers/test_lemonade-sdk.sh
```

Expected output — `test_ros.sh`:

```
Testing ROS 2 installation...
ROS_DISTRO: jazzy
ROS_VERSION: 2
ros2 found at: /opt/ros/jazzy/bin/ros2
Checking ROS 2 daemon...
Listing installed ROS 2 packages...
... (truncated, 374 total packages)
Testing ros2 topic list...
/parameter_events
/rosout
SUCCESS: ROS 2 installation test passed
```

Expected output — `test_o3de.sh`:

```
Testing O3DE installation...
o3de found at: /usr/local/bin/o3de
Verifying O3DE installation paths...
O3DE installed at: /opt/O3DE
Checking Vulkan support...
Vulkan Instance Version: 1.4.313
SUCCESS: O3DE installation test passed
```

Expected output — `test_rai.sh`:

```
==========================================
RAI Framework Test
==========================================
ROS 2 jazzy environment sourced

Testing RAI core import...
RAI core imports successful!

Testing RAI whoami import...
RAI whoami imports successful!

Testing ROS 2 integration...
ROS 2 CLI: available

RAI version: 2.12.1
ROS 2 distro: jazzy
Python 3.12.3
```

Expected output — `test_lemonade-sdk.sh` (downloads a small model on first run):

```
Starting lemond on default port 13305...
Server is ready!
Pulling model Llama-3.2-1B-Instruct-GGUF...
Model pulled successfully: Llama-3.2-1B-Instruct-GGUF
Testing completion API with model Llama-3.2-1B-Instruct-GGUF...
        "text": " The color of the sky appears blue to us because of the way
                   that light behaves when it enters Earth's atmosphere. ..."
✓ lemonade-sdk API test passed!
```
