# Map-3 isolated Nav2 MPPI fixed-obstacle gate: profiles v1 and v2

Date: 2026-10-02 (Asia/Taipei)

Status: **v2 passes one static-only route; the broader fixed-object and map-boundary gate remains open. No IMM pedestrian test or PPO training was run.**

## Scope and runtime boundary

- Controller runtime: Jetson AGX Orin, Ubuntu 20.04 host; isolated ARM64 image `isaac_ros_dev-aarch64:fixed`, ROS 2 Humble, Nav2 MPPI `1.1.12`, `ROS_DOMAIN_ID=178`, `ROS_LOCALHOST_ONLY=1`.
- Simulation: local Mac D1 Max kinematic MuJoCo proxy. The proxy pose was relayed to the isolated Nav2 bridge; Nav2 `controller_server` was the sole `/cmd_vel` publisher. The bridge was loopback-only, fail-closed, and disarmed after each episode.
- The container had read-only root, map and config mounts; it had no robot-device mounts. No package or image was installed or pulled. After completion the container was stopped but retained, and the temporary SSH tunnel was closed.
- Map-3 3x scene, one fixed map obstacle, pedestrian disabled, Unknown blocked, measured proxy radius `0.49 m`, independent required fixed-edge clearance `0.61 m`. Both profiles used the same 10.962 m Nav2 route, reset seed `10042`, start, goal, costmap, and static scene.
- This is controller-in-loop evidence for a kinematic proxy only. It is not a gait, dynamic-person, IMM, PPO, localization-accuracy, or real-robot result.

## Results

| Profile | Controller bounds | Outcome | Simulated time | Remaining goal distance | Minimum fixed-edge distance | Contact / map violation |
|---|---|---|---:|---:|---:|---|
| MPPI v1 | `vx_max=0.22 m/s`, `wz_max=0.60 rad/s` | Timeout | 70.000 s | 2.232 m | 0.741 m | none / none |
| MPPI v2 | `vx_max=0.30 m/s`, `wz_max=0.70 rad/s`; updated sampling and path/obstacle critic weights | Nav2 FollowPath succeeded | 39.328 s | 0.288 m | 0.931 m | none / none |

For v2, the local gate recorded 417 command samples, zero physical contact, zero geometric collision, zero map violation, and no timeout. The Nav2 action returned status `4` (succeeded). The minimum measured fixed-object edge distance exceeded the unchanged `0.61 m` acceptance floor by `0.321 m`.

This comparison is diagnostic rather than a single-variable causal experiment: the v2 profile changed the velocity caps, sampling parameters, and critic weights. It also compares Humble MPPI against the earlier Foxy/RPP baseline, so cross-controller timing is not a strict apples-to-apples benchmark. The next tuning experiment should vary one parameter family at a time and evaluate multiple matched seeds.

## Gate and map evidence

- Readiness passed: lifecycle nodes `map_server`, `planner_server`, and `controller_server` active; exactly one `/cmd_vel` publisher and one bridge subscriber; `/odom` and TF available; `robot_radius=0.61 m`; `track_unknown_space=true`; actual MPPI caps read back as `0.30/0.70`.
- Nav2 global path: 437 points, 10.962 m; all 654 dense geometry samples traversable; minimum static-box edge distance `0.869 m` before closed-loop execution.
- Costmap: `597 x 924` at `0.05 m/cell`; fixed-object center cost `254`; start cost `0`; `208,099` Unknown cells remain lethal; all 1,089 dense path/costmap samples clear.
- Source map identity and exported 3x map identity are preserved in `nav2_mppi_map3_seed10042_20261002/manifest.json`. The MuJoCo runner must receive the original source map as `--source-map-yaml` and the 3x Nav2 export as `--nav2-map-yaml`.

## Preserved evidence

In [`nav2_mppi_map3_seed10042_20261002/`](nav2_mppi_map3_seed10042_20261002/):

- v1 closed-loop metrics: `20261002_nav2_mppi_mujoco_fixed_gate_seed10042.json`.
- v2 corrected closed-loop metrics: `20261002_nav2_mppi_mujoco_fixed_gate_v2_retry_seed10042.json`.
- v2 Nav2 action result: `20261002_nav2_mppi_follow_path_v2_retry_seed10042.json`.
- v2 readiness, path, costmap JSON and raw bytes: files prefixed `20261002_mppi_readiness_v2_` and `20261002_nav2_mppi_*_v2_`.
- A pre-motion retry using the wrong MuJoCo source-map argument is also retained as `20261002_nav2_mppi_follow_path_v2_seed10042.json` (Nav2 aborted, status `6`). The local geometry assertion stopped the proxy before motion; this attempt is excluded from the controller result table. The corrected run used the original source map and the 3x map only for Nav2.

## Decision and next gate

The result supports continuing with the selected Nav2 MPPI runtime and doing bounded cost/parameter optimization later. It does not justify starting the 25k PPO pilot yet. First repeat fixed-object checks with mirrored placements and deliberate map-boundary approaches; then connect and validate IMM pedestrian predictions against the same independent collision/clearance gate. Preserve the v1 profile as a baseline and test future cost changes on multiple identical seeds, tracking success, timeout, collision, map violation, minimum clearance, path progress, and time.

Rollback is limited to stopping the isolated simulation container and bridge. No production Nav2 profile, source map, driver, motor, or robot command path was changed.
