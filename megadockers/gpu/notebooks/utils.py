import torch

@torch.no_grad()
def sanitize_norm(policy):
    # images
    buf_imgs = getattr(policy.normalize_inputs, "buffer_images", {})
    for k, buf in buf_imgs.items():
        for name in ["mean", "std"]:
            t = getattr(buf, name)
            t = t.clone()
            # replace non-finite
            t[~torch.isfinite(t)] = 0 if name == "mean" else 1
            # guard zero/near-zero std
            if name == "std":
                t = torch.where(t.abs() < 1e-6, torch.ones_like(t), t)
            setattr(buf, name, torch.nn.Parameter(t, requires_grad=False))

    # state
    m = policy.normalize_inputs.buffer_observation_state.mean.clone()
    s = policy.normalize_inputs.buffer_observation_state.std.clone()
    m[~torch.isfinite(m)] = 0
    s[~torch.isfinite(s)] = 1
    s = torch.where(s.abs() < 1e-6, torch.ones_like(s), s)
    policy.normalize_inputs.buffer_observation_state.mean = torch.nn.Parameter(m, requires_grad=False)
    policy.normalize_inputs.buffer_observation_state.std  = torch.nn.Parameter(s, requires_grad=False)