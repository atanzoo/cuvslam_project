# D1 Max proxy map-3 70 s path-only gate

Date: 2026-10-01 (Asia/Taipei)

Status: **FAILED EARLY**. Seed 10042 passed; seed 10043 timed out just outside goal tolerance. Per the stop rule, seeds 10044–10051 and all dual-target rollouts were not run. No PPO training was started.

## Boundary

- Local Mac project `.venv`; Gymnasium 1.3.0, MuJoCo 3.11.0, `MjSpec=True`.
- D1 Max kinematic proxy only; map scale 3, 10 m candidate-map route, 70 s cap, action 0 on each decision.
- Path-only condition: after reset, both generated trajectories were replaced with copies having `appear_time=inf`, obstacle bodies updated, tracker state cleared, and the environment re-observed. The simulator confirmed zero active generated actors throughout both completed episodes.
- This is not gait, real D1 Max geometry/dynamics acceptance, ROS/Nav2, Jetson, or hardware evidence.
- Existing worktree was already heavily dirty (including deletions, modifications, and untracked workstreams). Those changes were preserved. Only this evidence report and its evidence-index link were added; no source, map, checkpoint, or prior output was edited.

## Results

| Reset seed / curriculum | Episode / map scene seed | Route SHA-256 | Start → goal (m) | Arrival | Success | Collision | Timeout | Map violation | Final goal distance | Progress | Max absolute lateral error | Max abs heading error | Action 0 steps | Mode counts |
|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 10042 / 0 | 1369567760 / 1369567760 | `423c8d567a52e04fc51277845b39f6d88e27f31ae1f5dfb086a111e7ac7cd181` | `(-3.795000, -5.622817)` → `(-2.640837, 3.609163)` | 53.472 s | yes | no | no | no | 0.299901 m | 97.447% | 0.271891 m | 0.803006 rad | 557 | AVOID 367, CRUISE 190 |
| 10043 / 1 | 1015474421 / 1015474421 | `0ac5d4baca07a9eaebce7f1b029036506ff20436b5ab619e6f32b418810ee210` | `(-3.418654, -1.318654)` → `(-0.024542, 7.275458)` | — | **no** | no | **yes** | no | **0.331125 m** | 99.274% | 0.510598 m | 0.785398 rad | 730 | AVOID 608, CRUISE 122 |

Each seed generated one static object and one person before the path-only override. Their person motion families were `crossing` (seed 10042, curriculum 0) and `same_direction` (seed 10043, curriculum 1). Their generated actor specs were then disabled; no active actor was present during these gate rollouts. No action override was reported in either rollout. Modes are the environment's executed `info["mode"]` counts; action 0 was the requested decision for every step. AVOID may still be selected by the map-scene safety/planner logic with generated actors disabled.

The first seed met the 0.30 m goal criterion at 53.472 s. The second reached 99.274% path progress but ended at 0.331125 m, just outside the strict 0.30 m criterion, and timed out at the 70 s cap. This is a gate failure, not permission to proceed to dual-target validation or training.

## Preservation check

SHA-256 before and after the rollouts matched:

- `map.yaml`: `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d`
- `map.pgm`: `41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74`
- Existing 25k `policy.zip`: `45fdfb1d7da066d90f5df3b52c341aed35d92b49cd775530687757ce1f0f5f08`

No dual-target rollout, focused test, PPO training, Jetson/ROS runtime, or robot operation was performed in this phase.

## Reproduction command and harness

Run from the project root. This command is read-only with respect to source and protected assets. It reports each episode immediately and stops at the first failed path-only gate.

```bash
PYTHONPATH=simulation/path_planning/tools \
/Users/tsengpochien/Desktop/cuvslam_project/.venv/bin/python -u -c '<Python harness below>'
```

```python
import hashlib, json, traceback
from collections import Counter
from dataclasses import replace
import numpy as np
from simulation.path_planning.tools.d1_edu_sb3_env import D1DecisionEnv

MAP = "real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml"
COMMON = dict(
    profile="baseline", robot_profile="d1_max_proxy", kinematic_proxy=True,
    scenario_mode="people_balanced_multi_target", seed=0, route_length=10.0,
    mppi_samples=32, mppi_horizon=30, candidate_map_yaml=MAP,
    map_scene_scale=3.0, map_route_center_buffer_m=0.25,
)
all_pass = True
for trial in range(10):
    seed = 10042 + trial
    env = D1DecisionEnv(**COMMON, duration=70.0)
    try:
        _, reset_info = env.reset(seed=seed, options={"curriculum_index": trial})
        path = np.asarray(env.active_map_path, dtype=np.float64)
        identity = {
            "trial": trial, "reset_seed": seed,
            "episode_seed": reset_info["episode_seed"],
            "map_scene_seed": reset_info["map_scene_seed"],
            "curriculum_index": trial,
            "motion_family": reset_info["map_pedestrian_motion"],
            "start_xy": path[0].tolist(), "goal_xy": env.goal_world.tolist(),
            "route_m": float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
            "route_sha256": hashlib.sha256(path.tobytes()).hexdigest(),
            "generated_actors": env.config.scenario_obstacles,
        }
        env.trajectories = [replace(t, appear_time=float("inf"))
                            for t in env.trajectories]
        for body, trajectory in zip(env.obstacle_bodies, env.trajectories):
            body.trajectory = trajectory
        env.tracker.tracks.clear()
        env.tracker.next_track_id = 0
        env.tracker.selected_track = None
        env._update_bodies(0.0)
        env._observe()

        modes = Counter()
        steps = overrides = actors_active_max = 0
        max_lat = max_heading = 0.0
        terminated = truncated = False
        info = {}
        while not (terminated or truncated):
            _, _, terminated, truncated, info = env.step(0)
            steps += 1
            modes[str(info["mode"])] += 1
            overrides += int(bool(info.get("action_overridden", False)))
            max_lat = max(max_lat, abs(float(info.get("path_lateral_error", 0.0))))
            max_heading = max(max_heading, abs(float(info.get("path_heading_error", 0.0))))
            active = sum(bool(t.state(float(info["elapsed_time"]))[2])
                         for t in env.trajectories)
            actors_active_max = max(actors_active_max, active)

        success = bool(info.get("success", False))
        result = {
            **identity, "actors_active_max": actors_active_max,
            "steps": steps, "action_counts": {"CRUISE_action_0": steps},
            "action_overridden_steps": overrides, "mode_counts": dict(modes),
            "success": success, "collision": bool(info.get("collision", False)),
            "timeout": bool(info.get("timeout", False)),
            "map_violation": bool(info.get("map_violation", False)),
            "arrival_time_s": float(info["elapsed_time"])
                if info.get("reached_goal") else None,
            "terminal_goal_distance_m": float(info["distance_to_goal"]),
            "path_progress": float(info["path_progress"]),
            "max_abs_lateral_error_m": max_lat,
            "max_abs_heading_error_rad": max_heading,
            "terminal_mode": str(info["mode"]),
        }
        print(json.dumps(result, sort_keys=True, default=str), flush=True)
        if not success or result["collision"] or result["map_violation"]:
            print(json.dumps({"STOP": "path-only gate failed; dual-target stage not run",
                              "seed": seed}, sort_keys=True), flush=True)
            all_pass = False
            break
    except Exception as error:
        print(json.dumps({"STOP": "path-only reset/rollout error", "seed": seed,
                          "error": repr(error),
                          "traceback": traceback.format_exc()}, sort_keys=True), flush=True)
        all_pass = False
        break
    finally:
        env.close()
print(json.dumps({"path_only_gate": "PASS_10_OF_10" if all_pass else "FAILED_EARLY",
                  "dual_target_authorized_by_gate": all_pass}, sort_keys=True), flush=True)
```
