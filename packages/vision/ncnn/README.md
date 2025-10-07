# NCNN

This ryzer contains the configuration files necessary to build ncnn - a high-performance neural network inference computing framework optimized for mobile platforms.

## Building and Running the Docker Container

To build and run the Docker container:

```bash
ryzers build ncnn
ryzers run
```

You should see the following output indicating

```
xhost:  unable to open display ""
[0 AMD Radeon Graphics (RADV GFX1151)]  queueC=1[4]  queueT=0[1]
[0 AMD Radeon Graphics (RADV GFX1151)]  fp16-p/s/u/a=1/1/1/1  int8-p/s/u/a=1/1/1/1
[0 AMD Radeon Graphics (RADV GFX1151)]  subgroup=64(32~64)  ops=1/1/1/1/1/1/1/1/1/1
[0 AMD Radeon Graphics (RADV GFX1151)]  fp16-cm=16x16x16  int8-cm=16x16x16  bf16-cm=0  fp8-cm=0
[1 llvmpipe (LLVM 20.1.2, 256 bits)]  queueC=0[1]  queueT=0[1]
[1 llvmpipe (LLVM 20.1.2, 256 bits)]  fp16-p/s/u/a=1/1/1/1  int8-p/s/u/a=1/1/1/1
[1 llvmpipe (LLVM 20.1.2, 256 bits)]  subgroup=8(8~8)  ops=1/1/1/1/1/1/1/1/1/1
[1 llvmpipe (LLVM 20.1.2, 256 bits)]  fp16-cm=0  int8-cm=0  bf16-cm=0  fp8-cm=0
532 = 0.166382
920 = 0.094788
716 = 0.062683
```

To run the ncnn benchmark use the benchmark script

```
ryzers run /ryzers/benchmark.sh
```

It may take a couple minutes, but you'll see the following models evaluated:

```
[0 AMD Radeon Graphics (RADV GFX1151)]  queueC=1[4]  queueT=0[1]
[0 AMD Radeon Graphics (RADV GFX1151)]  fp16-p/s/u/a=1/1/1/1  int8-p/s/u/a=1/1/1/1
[0 AMD Radeon Graphics (RADV GFX1151)]  subgroup=64(32~64)  ops=1/1/1/1/1/1/1/1/1/1
[0 AMD Radeon Graphics (RADV GFX1151)]  fp16-cm=16x16x16  int8-cm=16x16x16  bf16-cm=0  fp8-cm=0
[1 llvmpipe (LLVM 20.1.2, 256 bits)]  queueC=0[1]  queueT=0[1]
[1 llvmpipe (LLVM 20.1.2, 256 bits)]  fp16-p/s/u/a=1/1/1/1  int8-p/s/u/a=1/1/1/1
[1 llvmpipe (LLVM 20.1.2, 256 bits)]  subgroup=8(8~8)  ops=1/1/1/1/1/1/1/1/1/1
[1 llvmpipe (LLVM 20.1.2, 256 bits)]  fp16-cm=0  int8-cm=0  bf16-cm=0  fp8-cm=0
loop_count = 10
num_threads = 32
powersave = 0
gpu_device = 0
cooling_down = 1
          squeezenet  min =    0.53  max =    0.55  avg =    0.54
           mobilenet  min =    0.48  max =    0.66  avg =    0.55
        mobilenet_v2  min =    0.69  max =    0.95  avg =    0.77
        mobilenet_v3  min =    0.82  max =    0.86  avg =    0.84
          shufflenet  min =    0.59  max =    0.75  avg =    0.61
       shufflenet_v2  min =    0.73  max =    0.77  avg =    0.75
             mnasnet  min =    0.69  max =    0.86  avg =    0.74
     proxylessnasnet  min =    0.75  max =    0.89  avg =    0.77
     efficientnet_b0  min =    1.35  max =    1.53  avg =    1.48
   efficientnetv2_b0  min =   17.94  max =   18.06  avg =   18.00
        regnety_400m  min =    1.02  max =    1.18  avg =    1.06
           blazeface  min =    0.56  max =    1.10  avg =    0.62
           googlenet  min =    1.54  max =    1.74  avg =    1.61
            resnet18  min =    0.92  max =    1.05  avg =    0.95
             alexnet  min =    1.26  max =    1.44  avg =    1.32
               vgg16  min =    3.43  max =    3.47  avg =    3.45
            resnet50  min =    1.88  max =    2.04  avg =    1.91
      squeezenet_ssd  min =    5.99  max =   16.98  avg =    8.13
       mobilenet_ssd  min =    2.04  max =    2.06  avg =    2.05
      mobilenet_yolo  min =    2.27  max =    2.37  avg =    2.32
  mobilenetv2_yolov3  min =    3.79  max =    4.33  avg =    3.97
         yolov4-tiny  min =    6.99  max =   17.99  avg =   10.65
           nanodet_m  min =    4.35  max =    4.77  avg =    4.52
    yolo-fastest-1.1  min =    1.97  max =    2.01  avg =    1.98
      yolo-fastestv2  min =    1.07  max =    1.28  avg =    1.13
  vision_transformer  min =   13.01  max =   13.67  avg =   13.22
          FastestDet  min =    1.02  max =    1.15  avg =    1.05
```

## Further Details

For more information, visit the official [NCNN repository](https://github.com/Tencent/ncnn).

Copyright(C) 2025 Advanced Micro Devices, Inc. All rights reserved.
