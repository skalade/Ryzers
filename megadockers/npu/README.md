# NPU Megadocker

## Host machine setup

Use the .deb files in xdna_driver to install the NPU driver version that matches ryzers

```
sudo dpkg -i xdna_driver/*.deb
```

## Build

```
docker build -t roscon_npu --build-arg ROS_DISTRO=kilted -f Dockerfile.npu .
```

## Run

```
docker run -it --rm \
  --shm-size 16G \
  --cap-add=SYS_PTRACE \
  --security-opt seccomp=unconfined \
  --ulimit memlock=-1 \
  --network=host \
  --ipc=host \
  -p 8888:8888 \
  -p 11311:11311 \
  -e HSA_OVERRIDE_GFX_VERSION=11.0.0 \
  -e OPENAI_API_KEY=your_api_key_here \
  -e RYZEN_AI_DIR=/ryzers/RyzenAI-SW/Ryzen-AI-CVML-Library \
  -e LD_LIBRARY_PATH=/ryzers/RyzenAI-SW/Ryzen-AI-CVML-Library/linux:/opt/xilinx/xrt/lib \
  -e DISPLAY=$DISPLAY \
  -v $PWD/mounted:/ryzers/results \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/kfd \
  --device=/dev/dri \
  --device=/dev/accel/accel0 \
  --group-add video \
  --group-add render \
  roscon_npu
```

## Test


Test XDNA
```
./test_xdna.sh
```

Expected output:
```
WARNING: User doesn't have admin permissions to set performance mode. Running validate in Default mode
Validate Device           : [0000:c6:00.1]
    Platform              : NPU Strix Halo
    Power Mode            : Default
-------------------------------------------------------------------------------
Test 1 [0000:c6:00.1]     : gemm                                                
    Details               : TOPS: 51.0
    Test Status           : [PASSED]
-------------------------------------------------------------------------------
Test 2 [0000:c6:00.1]     : latency                                             
    Details               : Average latency: 55.0 us
    Test Status           : [PASSED]
-------------------------------------------------------------------------------
Test 3 [0000:c6:00.1]     : throughput                                          
    Details               : Average throughput: 74992.0 op/s
    Test Status           : [PASSED]
-------------------------------------------------------------------------------
Validation completed. Please run the command '--verbose' option for more details

```

Test CVML:
```
test_cvml.sh
```

Expected output:
```
[INFO] time:22014659 thread:138873043959168 AMD CVML SDK: 0.0.0-dev
[INFO] time:22014659 thread:138873043959168 Any GPU inference will use AMD Radeon Graphics (RADV GFX1151)[0]
[INFO] time:22014659 thread:138873043959168 Any GPU inference will use AMD Radeon Graphics (RADV GFX1151)[0]
[INFO] time:22014667 thread:138873043959168 [Depth Estimation] Using ONNX engine, NPU backend
Opening video file: video_call.mp4
WARNING: Logging before InitGoogleLogging() is written to STDERR
I20251008 22:51:51.280300    26 vitisai_compile_model.cpp:1143] Vitis AI EP Load ONNX Model Success
I20251008 22:51:51.280366    26 vitisai_compile_model.cpp:1144] Graph Input Node Name/Shape (1)
I20251008 22:51:51.280372    26 vitisai_compile_model.cpp:1148] 	 efficient_Unet::input_0_nhwc : [1x256x256x3]
I20251008 22:51:51.280377    26 vitisai_compile_model.cpp:1154] Graph Output Node Name/Shape (1)
I20251008 22:51:51.280380    26 vitisai_compile_model.cpp:1158] 	 2196_nhwc : [1x256x256x1]
[Vitis AI EP] No. of Operators :   CPU     2    NPU   616 
[Vitis AI EP] No. of Subgraphs :   NPU     1 Actually running on NPU     1 
[INFO] time:22024573 thread:138872777221824 [ONNX VAI] Session created
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
Output file saved: /ryzers/results/output.mp4
```
