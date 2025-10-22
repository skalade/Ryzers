import torch
import matplotlib.pyplot as plt
import cv2

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


def plot_image(image_path: str):
    """Image plotting helper"""
    img = cv2.imread(image_path)
    fig, ax = plt.subplots(figsize=(4, 4))
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)