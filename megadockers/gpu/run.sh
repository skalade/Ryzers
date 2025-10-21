docker run -it --rm \
  --shm-size 16G \
  --cap-add=SYS_PTRACE \
  --security-opt seccomp=unconfined \
  --network=host \
  --ipc=host \
  -p 11434:11434 \
  -e AMDGPUTOP_TIMEOUT=10s \
  -e DISPLAY=$DISPLAY \
  -v $PWD/llamacpp_cache:/root/.cache \
  -v $PWD/workspace/.ollama:/root/.ollama/ \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $PWD/notebooks:/ryzers/notebooks \
  --device=/dev/kfd \
  --device=/dev/dri \
  --group-add video \
  --group-add render \
  roscon_gpu
