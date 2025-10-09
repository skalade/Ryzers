# NPU Megadocker

Run Flags:   -it --rm --shm-size 16G --cap-add=SYS_PTRACE  --network=host --ipc=host -e AMDGPUTOP_TIMEOUT=10s -v $PWD/llamacpp_cache:/root/.cache -p 11434:11434 -v $PWD/workspace/.ollama:/root/.ollama/ -e HSA_OVERRIDE_GFX_VERSION=11.0.0 --device=/dev/kfd --device=/dev/dri --security-opt seccomp=unconfined --group-add video --group-add render  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix

## Build

```
docker build -t roscon_gpu --build-arg GFXSTRING=gfx1151 -f Dockerfile.gpu .
```

## Run

```
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
  --device=/dev/kfd \
  --device=/dev/dri \
  --group-add video \
  --group-add render \
  roscon_gpu
```

## Test


Test amdgpu_top
```
./test_amdgputop.sh
```

Expected output:
```
amdgpu_top v0.11.0
ROCm Version: 7.0.0

--------
#0 DevicePath {
    render: "/dev/dri/renderD128",
    card: "/dev/dri/card1",
    accel: "",
    pci: "0000:c5:00.0",
    sysfs_path: "/sys/bus/pci/devices/0000:c5:00.0",

thread 'main' panicked at library/std/src/io/stdio.rs:1165:9:
failed printing to stdout: Broken pipe (os error 32)
note: run with `RUST_BACKTRACE=1` environment variable to display a backtrace
amdgpu_top test completed.
```

Test ACT
```
./test_act.sh
```

Expected output:
```
episode_idx=0
Rollout out EE space scripted policy
episode_idx=0 Failed
Replaying joint commands
episode_idx=0 Failed
Saving: 0.2 secs

Saved to data
Success: 0 / 1
```


Test Llama.cpp:
```
./test_llamacpp.sh
```

Expected output:
```

system_info: n_threads = 16 (n_threads_batch = 16) / 32 | ROCm : NO_VMM = 1 | PEER_MAX_BATCH_SIZE = 128 | CPU : SSE3 = 1 | SSSE3 = 1 | AVX = 1 | AVX_VNNI = 1 | AVX2 = 1 | F16C = 1 | FMA = 1 | BMI2 = 1 | AVX512 = 1 | AVX512_VBMI = 1 | AVX512_VNNI = 1 | AVX512_BF16 = 1 | LLAMAFILE = 1 | OPENMP = 1 | REPACK = 1 | 

sampler seed: 196817348
sampler params: 
	repeat_last_n = 64, repeat_penalty = 1.000, frequency_penalty = 0.000, presence_penalty = 0.000
	dry_multiplier = 0.000, dry_base = 1.750, dry_allowed_length = 2, dry_penalty_last_n = 4096
	top_k = 40, top_p = 0.950, min_p = 0.050, xtc_probability = 0.000, xtc_threshold = 0.100, typical_p = 1.000, top_n_sigma = -1.000, temp = 0.800
	mirostat = 0, mirostat_lr = 0.100, mirostat_ent = 5.000
sampler chain: logits -> logit-bias -> penalties -> dry -> top-n-sigma -> top-k -> typical -> top-p -> min-p -> xtc -> temp-ext -> dist 
generate: n_ctx = 4096, n_batch = 2048, n_predict = 100, n_keep = 0

a short story is the follows: “The story of the following is the following “the following” is the following “the” follows the following “the” is the following “the” is the” is the following “the” is follows “the following” is follows “the” follows “the” follows “the” follows “the” follows” follows “the” follows “the” follows “the” follows “the” follows “the” follows “the” follows “the” follows “the” follows “the

llama_perf_sampler_print:    sampling time =       3.51 ms /   107 runs   (    0.03 ms per token, 30501.71 tokens per second)
llama_perf_context_print:        load time =     957.49 ms
llama_perf_context_print: prompt eval time =      31.16 ms /     7 tokens (    4.45 ms per token,   224.68 tokens per second)
llama_perf_context_print:        eval time =    1416.87 ms /    99 runs   (   14.31 ms per token,    69.87 tokens per second)
llama_perf_context_print:       total time =    1457.34 ms /   106 tokens
llama_perf_context_print:    graphs reused =         98
llama_memory_breakdown_print: | memory breakdown [MiB] | total    free    self   model   context   compute    unaccounted |
llama_memory_breakdown_print: |   - ROCm0 (Graphics)   | 15862 = 12888 + (2660 =  1889 +     512 +     258) +         313 |
llama_memory_breakdown_print: |   - Host               |                   180 =   164 +       0 +      16                |
```

Test SmolVLA
```
HSA_OVERRIDE_GFX_VERSION=11.0.0 ./test_smolvla.sh
```

Expected output:
```
WARNING:root:Device 'None' is not available. Switching to 'cuda'.
Loading  HuggingFaceTB/SmolVLM2-500M-Video-Instruct weights ...
`torch_dtype` is deprecated! Use `dtype` instead!
generation_config.json: 100%|███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 136/136 [00:00<00:00, 964kB/s]
Reducing the number of VLM layers to 16 ...
model.safetensors: 100%|█████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 907M/907M [00:41<00:00, 22.0MB/s]
[standardise_state_dict] 'normalize_inputs.buffer_observation_state.mean'  ←  ['normalize_inputs.so100-red_buffer_observation_state.mean', 'normalize_inputs.so100_buffer_observation_state.mean']
[standardise_state_dict] 'normalize_inputs.buffer_observation_state.std'  ←  ['normalize_inputs.so100-red_buffer_observation_state.std', 'normalize_inputs.so100_buffer_observation_state.std']
[standardise_state_dict] 'normalize_targets.buffer_action.mean'  ←  ['normalize_targets.so100-red_buffer_action.mean', 'normalize_targets.so100_buffer_action.mean']
[standardise_state_dict] 'normalize_targets.buffer_action.std'  ←  ['normalize_targets.so100-red_buffer_action.std', 'normalize_targets.so100_buffer_action.std']
[standardise_state_dict] 'unnormalize_outputs.buffer_action.mean'  ←  ['unnormalize_outputs.so100-red_buffer_action.mean', 'unnormalize_outputs.so100_buffer_action.mean']
[standardise_state_dict] 'unnormalize_outputs.buffer_action.std'  ←  ['unnormalize_outputs.so100-red_buffer_action.std', 'unnormalize_outputs.so100_buffer_action.std']
/opt/venv/lib/python3.12/site-packages/transformers/integrations/sdpa_attention.py:96: UserWarning: Using AOTriton backend for Efficient Attention forward... (Triggered internally at /pytorch/aten/src/ATen/native/transformers/hip/attention.hip:1180.)
  attn_output = torch.nn.functional.scaled_dot_product_attention(
Chunk size: 50
Avg inference time: 0.104215 s
Avg inference hz: 479.775343 hz
Max GPU memory used: 937.21 MB
```
