# Reproducing the CaP-X Robosuite Task Grid on ROCm — Plan

This document is the concrete plan to extend the current `capx_experimental` Ryzer from "16 PyRoKi-only privileged configs verified" to "the full CaP-X Robosuite benchmark grid runnable on AMD Ryzen AI". It is the natural follow-on to `ROCM_PORTING.md` Tier 3.A.

The goal is **research-grade reproduction**: take the cap-x paper's Robosuite single-turn (S1–S4) and multi-turn (M1–M4) tier matrix, run it on ROCm, and have results that are statistically meaningful enough to build new methods on top of.

---

## Where we are today (TL;DR)

After the work in `0506_capx` / `capx_experimental`, this image runs:

| Coverage of cap-x's Robosuite grid | Tasks | Tiers |
|---|---|---|
| Verified end-to-end | 7 / 7 | privileged (oracle + LLM) |
| Verified end-to-end with `HF_TOKEN` + `SAM3_THRESHOLD=0.1` | 1 / 7 (spill_wipe) | unprivileged (oracle) |
| Probably runs, untested | 1 / 7 (two_arm_handover) | unprivileged |
| **Blocked** | 5 / 7 | unprivileged |

So out of `7 tasks × ~8 tiers ≈ 56 cells` in the headline benchmark grid, we cover roughly **7 (privileged) + 1 (spill_wipe unprivileged) ≈ 8 cells, ~14%**.

The 5 blocked tasks (cube_stack, cube_lifting, cube_restack, two_arm_lift, nut_assembly) are *all* gated on Contact-GraspNet. nut_assembly is doubly gated on molmo. **Porting Contact-GraspNet is the single intervention that turns ~14% coverage into ~85%.**

---

## Per-task blocker map

Determined by walking each config's `api_servers` block + each env class's actual imports (`grep init_molmo capx/integrations/franka/*.py`):

| Task | Privileged | Unprivileged blocker | Net blocker after GraspNet port |
|---|---|---|---|
| cube_stack | ✅ works (all variants — main / multiturn / vdm / reduced_api / skill_lib) | Contact-GraspNet | **none** |
| cube_lifting | ✅ works | Contact-GraspNet | **none** |
| cube_restack | ✅ works | Contact-GraspNet | **none** |
| two_arm_lift | ✅ works | Contact-GraspNet (+ OWL-ViT — pure HF, easy) | **none** (port OWL-ViT shim too) |
| spill_wipe | ✅ works | already runs (SAM3 + PyRoKi only) | already done |
| two_arm_handover | ✅ works | already unblocked (SAM3 + PyRoKi only); not yet end-to-end tested with an LLM | already done; needs an LLM bake-off |
| nut_assembly | ✅ works | molmo via `nut_assembly_visual.py` | **molmo (T2-A + T3-B)** |

So after a successful GraspNet port: **6 / 7 unprivileged tiers unlocked**. To get nut_assembly's seventh, we need either a working local Molmo backend (T2-A + T3-B) or an OpenRouter Molmo proxy if such a thing exists.

---

## The three GraspNet paths, ranked

These are not exclusive — start with #1, fall back to #2 if no fork delivers, save #3 as last resort.

### Path 1 — find an existing CPU/PyTorch/ROCm fork (15 min triage)

Cap-x pins `uynitsuj/contact_graspnet_pytorch`, which is a fork of Sundermeyer's TF original. There are several PyTorch ports in the wild; some have already removed the custom CUDA ops in favour of `torch.cdist` / `torch_cluster.knn` / `torch_geometric` equivalents.

**Triage checklist:**
- [ ] `gh search repos contact_graspnet pytorch` and skim for "rocm", "cpu", "no cuda" in READMEs.
- [ ] Check whether `uynitsuj/contact_graspnet_pytorch` itself has a CPU code path or a `--device cpu` flag — sometimes the CUDA dep is opt-in.
- [ ] Look at `Sundermeyer/contact_graspnet`'s issues/PRs for a "ROCm" or "CPU-only" branch.
- [ ] If anything looks promising, `pip install -e <fork>` into the capx Ryzer and run `python3 -m capx.serving.launch_contact_graspnet_server --device cuda` (PyTorch ROCm exposes ROCm as `cuda`). Watch for compile errors.

If this returns a working fork, drop it into the Dockerfile next to the SAM3 shim and we're done. Probability this works: ~30%.

### Path 2 — heuristic-grasp shim (~1 day, recommended default)

Cap-x's `launch_contact_graspnet_server.py` exposes a single endpoint that takes `(rgb, depth, mask, camera_intrinsics)` and returns a list of `(grasp_pose, score)` candidates. For tabletop manipulation, a serviceable approximation is:

```python
# pseudocode — see ROCM_PORTING.md T3-A for the longer sketch
def heuristic_grasps(rgb, depth, mask, intrinsics):
    centroid_uv = mask_centroid(mask)
    centroid_xyz = unproject(centroid_uv, depth, intrinsics)  # in camera frame
    centroid_world = camera_to_world(centroid_xyz)
    down_quat = np.array([1, 0, 0, 0])  # tool +Z pointing -Z world
    candidates = []
    for yaw in [0, np.pi/4, np.pi/2, 3*np.pi/4]:
        q = compose(down_quat, yaw_quat(yaw))
        candidates.append({"position": centroid_world, "quaternion_wxyz": q, "score": 0.9})
    return candidates
```

**Pros:**
- Zero kernel work, no new deps, runs on the existing image.
- Same JSON contract as upstream → no client-side patches in `capx/integrations/`.
- Likely good enough for cube_stack / cube_lifting / cube_restack / spill_wipe-style tabletop grasps where the empirical evidence (our spill_wipe run) shows that "bbox + downward orientation" is enough for the policy to plan a viable trajectory.

**Cons:**
- Not equivalent to GraspNet on tilted, occluded, or 6-DOF grasps. For two_arm_lift's bimanual coordination, may miss grasps that GraspNet would find.
- "Reproducing cap-x's baselines" means matching their numbers — if the heuristic systematically under- or over-shoots GraspNet, the reported success rates will diverge from upstream and the comparison loses meaning.

**Mitigation:** treat the heuristic shim as a *baseline*, not a replacement. Document it as `launch_grasp_heuristic_server.py` (different name from the upstream filename) so it's clear this is not GraspNet-equivalent. Validate by running the same benchmark on NVIDIA hardware with real GraspNet for at least one task; if heuristic performance is within ~10pp of GraspNet on cube_stack, we have a defensible substitute. If it's 30pp worse, fall back to Path 3.

### Path 3 — PyTorch-native PointNet++ port (~3–7 days)

Replace the custom CUDA ops with PyTorch / `torch_geometric` / `torch_cluster` equivalents. Affected ops are roughly:
- ball-query (for set abstraction layers) → `torch_cluster.radius`
- KNN → `torch_cluster.knn`
- furthest-point sampling → either reference Python or `torch_cluster.fps`
- group-points → `torch.gather`-based reimplementation

`torch_cluster` and `torch_geometric` both have ROCm builds, but installing them on top of the pinned base image's torch is non-trivial (they're sensitive to torch ABI). Plan for a separate sandbox to pin compatible versions before integrating.

**When to take this path:** only if Paths 1 and 2 fail or are inadequate. The win is "matches GraspNet behavior with high confidence"; the cost is real engineering time and a non-trivial new dep tree to manage.

---

## Validation plan — how we know the port works

Don't accept the port until *all* of these pass:

1. **Server boots and answers `/health`** with `device=cuda` (i.e. ROCm).
2. **JSON contract test**: send a synthetic `(rgb, depth, mask)` request, assert the response decodes through `capx/integrations/perception/contact_graspnet.py` (or wherever the cap-x client lives) into a list of `(pos, quat, score)`.
3. **Single-trial cube_stack non-privileged**: run `franka_robosuite_cube_stack.yaml` (the *default*, not `_privileged`) with `--use-oracle-code True --total-trials 1`. Expect the env to step without sandbox errors.
4. **3-trial cube_stack non-privileged with an LLM**: run with `gpt-4o-mini` or similar; expect ≥1 successful trial. If 0/3, the heuristic is too noisy or GraspNet is malfunctioning.
5. **Per-task quick bake-off**: 3 trials each of cube_stack / cube_lifting / cube_restack / two_arm_lift in their default (unprivileged) configs. Numbers don't have to be amazing; they just have to be non-zero on at least cube_stack and cube_lifting.

If all 5 pass, the port is good enough to begin reproducing cap-x's grid. If only 1–3 pass, we have a partial unlock — document the limitation and proceed with the subset of tasks that work.

---

## Staged execution plan

A single-agent-session-sized work breakdown. Each stage commits independently so progress is preserved.

### Stage 1 — Path 1 triage (~30 min)

- [ ] Search for existing CPU/PyTorch/ROCm forks of contact_graspnet (5 min).
- [ ] Sandbox-install the most promising candidate (10 min).
- [ ] Run `python3 -m capx.serving.launch_contact_graspnet_server` against it; smoke test `/health`.
- [ ] If a fork works: skip to Stage 4 (integration). If not: continue.

### Stage 2 — Heuristic shim (Path 2) (~1 day)

- [ ] Write `packages/robotics/capx/launch_grasp_heuristic_server.py` mirroring the SAM3 shim pattern (FastAPI on a configurable port; same JSON contract as upstream `launch_contact_graspnet_server`).
- [ ] Add `Dockerfile` overlay that **only loads** the heuristic server when an env var (e.g. `CAPX_USE_HEURISTIC_GRASP=1`) is set, so we can A/B against any future real-GraspNet port without a rebuild.
- [ ] Add `demo_grasp.sh` that round-trips a synthetic image through the heuristic server and decodes the response.
- [ ] Update `test.sh` with an import + `__doc__` assertion (same defensive pattern as the SAM3 shim).
- [ ] Update `Dockerfile` to copy the new shim. Rebuild. Confirm the smoke test passes.

### Stage 3 — End-to-end validation on cube_stack non-privileged (~1 day)

- [ ] Run `franka_robosuite_cube_stack.yaml` (default, not _privileged) with `--use-oracle-code True`. Expect at least sandbox-success.
- [ ] Run with an LLM (`gpt-4o-mini` via OpenRouter). Expect ≥1/3 trial success on cube_stack.
- [ ] If success rate is acceptable on cube_stack → repeat for cube_lifting, cube_restack, two_arm_lift, two_arm_handover.
- [ ] Document numbers in `ROCM_PORTING.md` Tier 3.A and append to README's benchmark table.

### Stage 4 — Full grid run (~2–3 days, mostly compute time)

- [ ] Pick the LLM(s) for the headline run. `gemini-2.5-flash` led the privileged bake-off; ideally include one local LLM (depends on T2-A) for full reproducibility.
- [ ] Build a `reproduce_robosuite_grid.sh` runner (similar to `bench_capx.sh`) that iterates `task × tier × LLM` and writes results to a structured CSV.
- [ ] Run with `n ≥ 30 trials/cell`. Privileged tier is fast (~10s/trial); unprivileged is slower (~60s/trial including SAM3 + grasp + IK + sim).
- [ ] Generate a results table comparable to cap-x's headline figure.
- [ ] (Optional) NVIDIA reference run for cross-vendor delta.

### Stage 5 — Document & ship (~half day)

- [ ] Update `README.md` "what works" table to reflect ~85% Robosuite-grid coverage.
- [ ] Append final benchmark numbers to `ROCM_PORTING.md`.
- [ ] Tag the `capx_experimental` branch with a release marker (e.g. `capx_experimental_v0.1`) so downstream research builds against a stable baseline.

---

## What this does not unlock (for honesty)

Even with a working GraspNet port, these remain blocked:
- **LIBERO**: still needs the Max-Fu/robosuite fork chained as a separate Ryzer (T1-C). The grasp port helps, but it's not sufficient. Plan: a chained `capx_libero_experimental` Ryzer for a future session.
- **r1pro / BEHAVIOR**: NVIDIA Isaac Sim, permanent block. Not in cap-x's Robosuite grid anyway.
- **verl-RL training**: needs vLLM-ROCm + flash-attn-ROCm. Different problem; orthogonal to grid reproduction.
- **nut_assembly unprivileged**: still needs molmo. Either run with 6/7 tasks reported, or invest in T2-A/T3-B.

---

## Risks & known unknowns

- **The heuristic shim might tank cube_stack / cube_lifting success rates** to the point where LLMs can't recover. We have one data point in favor (spill_wipe runs end-to-end with bbox-only perception), but cube manipulation needs more orientation info than spill wiping. Sandbox before committing.
- **SAM3 mask quality on robosuite scenes** is workload-dependent. Cube objects worked at score 0.94+; spill_wipe needed `SAM3_THRESHOLD=0.1`. Other tasks (lifting, restack) may have their own threshold tuning needs. Plan: add per-task threshold overrides to the bench runner.
- **`SAM3_THRESHOLD=0.1` reduces precision**, not just recall. We may get spurious masks for cluttered scenes (two_arm_lift). Validate the threshold per task.
- **Cap-x's grid uses specific LLMs** (gpt-4o, claude-3.5-sonnet, gemini-1.5-pro). Tier-0 models like `gpt-4o-mini` are cheaper but score lower. To match upstream numbers, eventually run with the original models; budget accordingly.
- **NVIDIA reference numbers**: without these, "we reproduced cap-x's grid on ROCm" has no baseline to compare against. Either pull from cap-x's paper directly (caveat: different test seed, different LLM versions) or rent NVIDIA time.

---

## Concrete next handoff

The next agent session should:

1. Read this doc and `ROCM_PORTING.md` Tier 3.A.
2. Execute Stage 1 (15 min) and report whether an existing fork works.
3. If no, execute Stage 2 (write the heuristic shim).
4. Run Stage 3 validation. Stop after cube_stack + cube_lifting numbers — don't grind the full grid until the heuristic is validated as good enough on at least two tasks.
5. Commit each stage independently on `capx_experimental`.

Once Stage 3 numbers look reasonable, the user is in position to either (a) start building research on top, or (b) escalate to Path 3 (PyTorch-native port) if heuristic fidelity isn't enough.
