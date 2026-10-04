# D1 Max proxy map-3 dual-target balanced 70 s fresh 25k PPO

Date: 2026-10-01 (Asia/Taipei)  
Status: **stopped early at 11,520 recorded steps after the agreed collision gate**.  
Change class: C3 local simulation experiment; no source/controller edit in this run.

## Boundary and protected assets

Local Mac CPU, MuJoCo D1 Max kinematic proxy, Stable-Baselines3 PPO. This is
not gait, real D1 Max dynamics, ROS/Nav2, Jetson, hardware, or safety evidence.
No Jetson/robot/runtime access, source edits, map edits, commits, pushes, or
old-model initialization are part of this run. The existing 25k policy and
prior output remain the baseline, not a compatible checkpoint under the
changed map-3 route/controller revision.

Candidate map: `real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml`.

| Protected file | SHA-256 before training |
|---|---|
| `map.yaml` | `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d` |
| `map.pgm` | `41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74` |
| prior `policy.zip` | `45fdfb1d7da066d90f5df3b52c341aed35d92b49cd775530687757ce1f0f5f08` |

The new output root was absent before launch:
`output/d1_max_proxy_map3_dual_target_balanced_70s_25k_20261001/`.
Available disk immediately before training was 26 GiB. Activity Monitor search
for `run_d1_edu_decision_layer_sb3` showed no matching process. The shell
process-list APIs were unavailable in this environment; the Activity Monitor
check is the available conflict check.

## Environment

- Python 3.14.6; NumPy 2.5.1; PyTorch 2.13.0; SB3 2.9.0.
- Gymnasium 1.3.0; MuJoCo 3.11.0; `MjSpec=True`.
- Map geometry scale 3, 0.05 m source resolution, 10 m routes.
- D1 Max kinematic proxy; `full_randomization`; balanced pedestrian families;
  one static object and one person per scene.
- Training/evaluation episode duration 70 s; MPPI 32 samples x 30 horizon;
  collision penalty 20, timeout penalty 4; PPO `n_steps=256`, batch size 64,
  seed 42; 25,000 requested steps (expected 25,088 at a full rollout boundary).
- Fresh model only; no `--load-model` or `--resume-model`.
- The runner's default cruise mode is `goal_hysteresis`; no safety or failure
  checks were relaxed.

## First-step scene-validity gate

Validated reset seeds `10042`–`10051`, curriculum indices `0`–`9`, with the
project `.venv`. For each scene, repeating the same reset produced identical
episode/map-scene identity, route bytes, actors, and finite observation.
Every scene had exactly one static object and one person; every route was 10 m
and entirely in both the unchanged 13-cell runtime traversability mask and the
stricter route-selection mask. Goal and start were traversable. The fixed
object's dense footprint samples were known-free; its center remained at least
1.386 m from the goal and 1.500 m from the start, versus 0.844 m required for
proxy-radius plus object circumscribed-radius clearance. A bypass was valid on
at least one side of every fixed object at a 0.964 m center offset, including
the existing obstacle margin.

The active person trajectory was sampled every 0.1 s through 70 s; every active
sample was finite and in the runtime traversable mask, and did not overlap the
fixed object. Across the original ten scenes, closest person/object center
separation was at least 1.973 m, above the 0.807 m combined circumscribed
radius-plus-buffer check. All five families were present in curriculum order:
crossing, same-direction, accelerating, turning, irregular, then repeated on
the other side. A paired same-scene check confirmed mirrored signs for all
five families, including the turning center. The original-seed table:

| Reset seed | Map-scene seed | Pedestrian family |
|---:|---:|---|
| 10042 | 1369567760 | crossing |
| 10043 | 1015474421 | same-direction |
| 10044 | 294304262 | accelerating |
| 10045 | 608638769 | turning |
| 10046 | 1158153928 | irregular |
| 10047 | 155865358 | crossing |
| 10048 | 2007815779 | same-direction |
| 10049 | 482180392 | accelerating |
| 10050 | 1321107238 | turning |
| 10051 | 1413292347 | irregular |

Verdict: **scene-validity gate PASS**. Per the user's revised instruction,
path-only completion is not a prerequisite; the earlier 70 s path-only gate
failure remains separate evidence and was not overwritten.

## Focused regression

Command:

```bash
PYTHONPATH=simulation/path_planning/tools \
.venv/bin/python simulation/path_planning/tools/test_d1_edu_decision_layer_sb3.py
```

Result: PASS (all five printed checks): SB3 reset/step, D1 Max proxy reset/step,
3x/10 m dual-target curriculum and mask semantics, per-5k outcome aggregation,
and map identity/scale checkpoint guard.

## Exact training command

```bash
PYTHONPATH=simulation/path_planning/tools \
.venv/bin/python -u simulation/path_planning/tools/run_d1_edu_decision_layer_sb3.py \
  --timesteps 25000 --eval-trials 10 \
  --profile full_randomization \
  --scenario-mode people_balanced_multi_target --seed 42 \
  --robot-profile d1_max_proxy --kinematic-proxy \
  --route-length 10 \
  --candidate-map-yaml real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml \
  --map-scene-scale 3 \
  --train-duration 70 --eval-duration 70 \
  --mppi-samples 32 --mppi-horizon 30 \
  --collision-penalty 20 --timeout-penalty 4 \
  --n-steps 256 --batch-size 64 \
  --policy output/d1_max_proxy_map3_dual_target_balanced_70s_25k_20261001/policy.zip \
  --summary output/d1_max_proxy_map3_dual_target_balanced_70s_25k_20261001/summary.json \
  --tensorboard-log output/d1_max_proxy_map3_dual_target_balanced_70s_25k_20261001/tensorboard \
  --checkpoint-freq 5000 \
  --checkpoint-dir output/d1_max_proxy_map3_dual_target_balanced_70s_25k_20261001/checkpoints \
  --verbose
```

Requested run limit: 25k steps, maximum 90 min wall clock. The main agent
applied the pre-agreed early-stop gate after the second outcome window showed
severe collision deterioration. No paired evaluation was run before stopping.

## Early-stop result

Only the exact PPO execution session `77040` received Ctrl-C. The child exited
with Python `KeyboardInterrupt` in the next `env.step()` while MuJoCo was in
`mj_forward`; Activity Monitor search for
`run_d1_edu_decision_layer_sb3` then showed no matching process. TensorBoard's
last fully recorded SB3 step is **11,520** (`time_elapsed=552 s`, about 9 min
12 s); the interrupt happened during the following environment step, so any
partially advanced unrecorded step is not counted. There was no restart,
resume, deletion, or signal to another process.

| Training window | Completed episodes | Success | Collision | Timeout | Map violation | Rolling mean reward |
|---|---:|---:|---:|---:|---|---:|
| 0–5k (logged at 5,120) | 11 | 0/11 (0%) | 10/11 (90.9%) | 0/11 (0%) | Not recorded by runner window callback | -32.8 |
| 5k–10k (logged at 10,240) | 11 | 0/11 (0%) | 11/11 (100%) | 0/11 (0%) | Not recorded by runner window callback | -30.1 |

Combined: **0/22 successes, 21/22 collisions, 0/22 timeouts** in the two
completed windows. The runner's 5k callback logs success/collision/timeout but
not map violations; no source change was made to add that metric. The last
rolling `rollout/ep_rew_mean` was `-29.8264` at step 11,520. Reward moved in a
better direction while collision outcomes were unacceptable; reward is not a
substitute for the outcome gate.

The saved checkpoint and event artifacts are:

| Artifact | SHA-256 |
|---|---|
| `checkpoints/policy.zip_5000_steps.zip` | `548b712752cfce8cdeded7ca668515bcafd611b2889cc046fb0ec292e42d51bb` |
| `checkpoints/policy.zip_10000_steps.zip` | `4b1e1da8733fb51d0001dc1ca266c9c1c3354ef4ab02812d0b8d0620501d7a62` |
| `tensorboard/PPO_1/events.out.tfevents.1790834193.MacBook-Pro.local.11724.0` | `bb7d21504829ca6055b1517163d46439a545c0a9180a5b853e8dd531cceba7c0` |

These files remain under
`output/d1_max_proxy_map3_dual_target_balanced_70s_25k_20261001/`. No final
`policy.zip`, metadata sidecar, or summary was written because training did
not finish. No per-episode collision trace/state was persisted, so collision
context cannot be reconstructed from the saved checkpoints/TensorBoard alone
without running an evaluation; none was started. The map YAML/PGM and old
25k policy hashes were rechecked after stopping and remained identical to the
pre-training values above. Do not resume either checkpoint unless the user
explicitly approves a new diagnostic/training plan.

## Post-stop matched sample evaluation (offline diagnostic only)

After the safe stop, five matched baseline/checkpoint samples were selected
to address the user's request for post-training inspection. This is a new
evaluation, not training or resumption, and these are
not represented as the exact episodes seen during training. The saved 10k
checkpoint was loaded directly with SB3 `PPO.load(..., env=..., device="cpu")`;
the runner metadata guard was intentionally bypassed because this checkpoint
has no metadata sidecar. It is not thereby validated as compatible or
deployable. `model.num_timesteps` reported 10,000. Checkpoint SHA-256 matched
the saved artifact above: `4b1e1da8733fb51d0001dc1ca266c9c1c3354ef4ab02812d0b8d0620501d7a62`.

The five reset seeds were 10042–10046 with curriculum indices 0–4. For each
seed, action-0 baseline and deterministic checkpoint evaluation were reset
separately in the same environment configuration. Before interpreting a pair,
the harness required exact equality of episode seed, map-scene seed, curriculum
index/family, route array SHA-256, start/goal, route length, every actor field,
and map scale. All five pairs passed identity equality and produced the
expected family order. Every route was 10 m; the actor set was exactly one
static `object` plus one `person`. No scene-identity mismatch or invalid scene
occurred, so the requested five-seed limit was reached.

| Seed / family | Policy | Result | Elapsed (s) | Final / min goal dist (m) | Max path progress | Min clearance (m) | Min proxy clearance (m) | Actions (steps) | Modes (steps) |
|---|---|---|---:|---:|---:|---:|---:|---|---|
| 10042 crossing | action-0 | collision + physical contact | 34.880 | 2.041 | 0.779 | -0.029 | -0.124 | CRUISE 364 | CRUISE 165, AVOID 199 |
| 10042 crossing | 10k deterministic | collision + physical contact | 46.824 | 2.037 | 0.779 | -0.028 | -0.119 | SLOWDOWN 342, CRUISE 146 | CRUISE 219, AVOID 269 |
| 10043 same-direction | action-0 | collision + physical contact | 28.288 | 5.314 | 0.437 | -0.022 | -0.033 | CRUISE 295 | CRUISE 97, AVOID 198 |
| 10043 same-direction | 10k deterministic | collision + physical contact | 32.928 | 5.290 | 0.437 | -0.039 | -0.056 | SLOWDOWN 212, CRUISE 131 | CRUISE 135, AVOID 208 |
| 10044 accelerating | action-0 | collision + physical contact | 35.848 | 3.752 | 0.612 | -0.023 | -0.035 | CRUISE 374 | CRUISE 105, AVOID 269 |
| 10044 accelerating | 10k deterministic | collision + physical contact | 44.272 | 3.747 | 0.612 | -0.027 | -0.039 | SLOWDOWN 232, CRUISE 230 | CRUISE 153, AVOID 309 |
| 10045 turning | action-0 | collision + physical contact | 30.544 | 5.315 | 0.437 | -0.021 | -0.032 | CRUISE 319 | CRUISE 98, AVOID 221 |
| 10045 turning | 10k deterministic | collision + physical contact | 33.536 | 5.295 | 0.437 | -0.035 | -0.051 | SLOWDOWN 257, CRUISE 93 | CRUISE 167, AVOID 183 |
| 10046 irregular | action-0 | collision + physical contact | 55.552 | 2.069 | 0.778 | -0.022 | -0.109 | CRUISE 579 | CRUISE 51, AVOID 528 |
| 10046 irregular | 10k deterministic | collision + physical contact | 58.520 | 2.077 | 0.779 | -0.028 | -0.129 | SLOWDOWN 304, CRUISE 306 | CRUISE 60, AVOID 550 |

All episodes terminated on collision, so the minimum goal distance equals the
terminal distance. Collision context below records the robot pose and nearest
active actor center at the terminal collision step; the elapsed time above is
the collision time. Center distance is contextual only, not a geom-pair
contact attribution.

| Seed / family | Policy | Collision-time robot XY (m) | Nearest active actor | Actor-center distance (m) |
|---|---|---|---|---:|
| 10042 crossing | action-0 | (-3.036, 1.607) | fixed object (-3.145, 2.318) | 0.720 |
| 10042 crossing | 10k deterministic | (-3.010, 1.606) | fixed object (-3.145, 2.318) | 0.725 |
| 10043 same-direction | action-0 | (-3.096, 2.939) | fixed object (-2.641, 3.609) | 0.810 |
| 10043 same-direction | 10k deterministic | (-3.054, 2.939) | fixed object (-2.641, 3.609) | 0.788 |
| 10044 accelerating | action-0 | (-3.091, 2.937) | fixed object (-2.641, 3.609) | 0.809 |
| 10044 accelerating | 10k deterministic | (-3.083, 2.938) | fixed object (-2.641, 3.609) | 0.804 |
| 10045 turning | action-0 | (-3.097, 2.938) | fixed object (-2.641, 3.609) | 0.812 |
| 10045 turning | 10k deterministic | (-3.062, 2.938) | fixed object (-2.641, 3.609) | 0.792 |
| 10046 irregular | action-0 | (0.756, 8.796) | fixed object (0.605, 9.515) | 0.734 |
| 10046 irregular | 10k deterministic | (0.659, 8.802) | fixed object (0.605, 9.515) | 0.714 |

All 10 evaluated rollouts ended in collision before timeout; none succeeded or
violated the map boundary. All 10 reported geometric collision, proxy collision,
and physical contact. Checkpoint action counts contained only CRUISE and
SLOWDOWN; the environment's safety/controller mode was CRUISE or AVOID in all
samples, with no action override. In each sample, the terminal robot position
was about 0.71–0.81 m center-to-center from the fixed object and farther from
the pedestrian (the pedestrian had already disappeared in several motion
families). This is consistent with the static obstacle being the nearby
collision context; the harness did not capture geom-pair contact identity, so
it does not claim that the fixed object caused every physical contact.

The PPO policy ran longer than action-0 in all five cases, but terminal
progress was effectively the same within each pair; it did not prevent the
shared collision pattern. This small, deliberately bounded sample is a
diagnostic signal only—not a success-rate estimate, a statistically powered
comparison, or evidence of generalization. It provides no gait, ROS/Nav2,
Jetson, hardware, or real-robot claim. The 10k checkpoint remains bound to its
old training-time identity gap and must not be treated as equivalent to a
model trained under a different route/controller revision.

### Reproduction command and harness

The following was run from the project root. It writes no files and performs
only five paired 70 s offline rollouts. `PPO.load` is direct by design; do not
replace it with the runner's guarded `--load-model` path or interpret this as
checkpoint acceptance.

```bash
PYTHONPATH=simulation/path_planning/tools .venv/bin/python -u - <<'PY'
import hashlib
from collections import Counter
import numpy as np
from stable_baselines3 import PPO
from d1_edu_sb3_env import D1DecisionEnv, ACTION_NAMES

MAP = "real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml"
CKPT = "output/d1_max_proxy_map3_dual_target_balanced_70s_25k_20261001/checkpoints/policy.zip_10000_steps.zip"
FAMILIES = ("crossing", "same_direction", "accelerating", "turning", "irregular")

def identity(env, reset_info):
    route = np.asarray(env.active_map_path, dtype=np.float64)
    return {
        "episode_seed": int(reset_info["episode_seed"]),
        "map_scene_seed": int(reset_info["map_scene_seed"]),
        "curriculum_index": int(reset_info["map_curriculum_index"]),
        "family": reset_info["map_pedestrian_motion"],
        "route_sha256": hashlib.sha256(route.tobytes()).hexdigest(),
        "start_xy": route[0].tolist(), "goal_xy": route[-1].tolist(),
        "route_length_m": float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum()),
        "actors": [dict(a.__dict__) for a in env.trajectories],
        "map_scale": float(reset_info["map_scene_scale"]),
    }

def rollout(env, seed, curriculum, model=None):
    obs, reset_info = env.reset(seed=seed, options={"curriculum_index": curriculum})
    scene = identity(env, reset_info)
    initial_goal = float(np.linalg.norm(env.data.qpos[:2] - env.goal_world))
    min_goal, max_progress = initial_goal, 0.0
    min_clearance = min_proxy_clearance = float("inf")
    actions, modes = Counter(), Counter()
    context = None
    while True:
        if model is None:
            action = 0
        else:
            predicted, _ = model.predict(np.asarray(obs, dtype=np.float32), deterministic=True)
            action = int(np.asarray(predicted).reshape(-1)[0])
        actions[ACTION_NAMES[action]] += 1
        obs, _, terminated, truncated, info = env.step(action)
        modes[str(info.get("mode", "?"))] += 1
        xy = env.data.qpos[:2].copy()
        min_goal = min(min_goal, float(np.linalg.norm(xy - env.goal_world)))
        max_progress = max(max_progress, float(info.get("path_progress", 0.0)))
        min_clearance = min(min_clearance, float(info.get("min_clearance", float("inf"))))
        min_proxy_clearance = min(min_proxy_clearance, float(info.get("min_proxy_clearance", float("inf"))))
        if (info.get("collision") or info.get("map_violation")) and context is None:
            near = []
            for actor in env.trajectories:
                position, _, active = actor.state(float(env.elapsed))
                if active:
                    near.append({"type": actor.kind, "motion_kind": actor.motion_kind,
                        "distance_center_m": float(np.linalg.norm(xy-position)),
                        "position_xy": position.tolist()})
            context = {"time_s": float(env.elapsed), "robot_xy": xy.tolist(),
                       "nearest_active_actors": sorted(near, key=lambda x: x["distance_center_m"])}
        if terminated or truncated:
            break
    result = {
        "success": bool(info.get("success", False)), "collision": bool(info.get("collision", False)),
        "timeout": bool(info.get("timeout", False)), "map_violation": bool(info.get("map_violation", False)),
        "elapsed_s": float(info.get("elapsed_time", env.elapsed)),
        "final_goal_distance_m": float(np.linalg.norm(env.data.qpos[:2]-env.goal_world)),
        "min_goal_distance_m": min_goal, "final_path_progress": float(info.get("path_progress", 0.0)),
        "max_path_progress": max_progress, "min_clearance_m": min_clearance,
        "min_proxy_clearance_m": min_proxy_clearance,
        "physical_contact": bool(info.get("physical_contact", False)),
        "geometric_collision": bool(info.get("geometric_collision", False)),
        "proxy_collision": bool(info.get("proxy_collision", False)),
        "steps": sum(actions.values()), "action_counts": dict(actions),
        "mode_counts": dict(modes), "collision_context": context,
    }
    return scene, result

env = D1DecisionEnv(profile="full_randomization", robot_profile="d1_max_proxy",
    kinematic_proxy=True, scenario_mode="people_balanced_multi_target", seed=42,
    duration=70, route_length=10, mppi_samples=32, mppi_horizon=30,
    collision_penalty=20, timeout_penalty=4, cruise_mode="goal_hysteresis",
    candidate_map_yaml=MAP, map_scene_scale=3)
model = PPO.load(CKPT, env=env, device="cpu")
try:
    for trial, seed in enumerate(range(10042, 10047)):
        base_scene, baseline = rollout(env, seed, trial)
        ppo_scene, checkpoint = rollout(env, seed, trial, model)
        if base_scene != ppo_scene:
            raise RuntimeError(f"scene identity mismatch at seed {seed}")
        if base_scene["curriculum_index"] != trial or base_scene["family"] != FAMILIES[trial]:
            raise RuntimeError(f"invalid scene identity at seed {seed}: {base_scene}")
        print({"trial": trial, "seed": seed, "identity": base_scene,
               "baseline": baseline, "checkpoint_10k": checkpoint}, flush=True)
finally:
    env.close()
PY
```

Latest hash recheck after evaluation:

| File | SHA-256 |
|---|---|
| `tools/d1_edu_sb3_env.py` | `97571573f60aed02152bb670f70968f2063cbbb27caf6b49c9b0423b15faf484` |
| `tools/d1_edu_nav2_global_path.py` | `cb90854493842887723eca9d924d36b9f91f44c98b1aeb00f46cc4d0f8ea735f` |
| `tools/run_d1_edu_decision_layer_sb3.py` | `1b9134afbcc616567b8ab6b543dbd76f6dd416936b4aa143304cea7daa9f546f` |
| map `map.yaml` | `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d` |
| map `map.pgm` | `41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74` |
| stopped-run 10k checkpoint | `4b1e1da8733fb51d0001dc1ca266c9c1c3354ef4ab02812d0b8d0620501d7a62` |

No source, map, checkpoint, or output artifact was modified by this sample
evaluation. The only new write for this follow-up is this appended evidence
section. The evaluation does not change the previous early-stop verdict: do
not resume/train from this checkpoint without a separately approved plan.
