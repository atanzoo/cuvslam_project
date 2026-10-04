# Map-3 isolated Nav2 RPP controller-in-loop fixed-obstacle gates

Date: 2026-10-02 (Asia/Taipei)

Status: **PASS for two directions in one fixed-obstacle scene only. Broader boundary/mirror and pedestrian gates remain open; no PPO training started.**

## Scope and runtime boundary

- Nav2 ran on Jetson AGX Orin, Ubuntu 20.04 / ROS 2 Foxy, in isolated worktree `/home/tseng/jetson_slam_ws/simulation/path_planning/nav2_map3_20261001`, with `ROS_DOMAIN_ID=43` and `ROS_LOCALHOST_ONLY=1`. The isolated map, planner, controller, and bridge were used; production workspace, robot drivers, motors, and real `/cmd_vel` consumers were not involved.
- The local Mac ran the D1 Max **kinematic** MuJoCo proxy. Its pose was fed to the isolated Nav2 bridge, and Nav2 `controller_server` owned `/cmd_vel`; the guarded bridge passed those commands to the proxy. This is not a gait, dynamics-identification, or real-robot test.
- Controller: `nav2_regulated_pure_pursuit_controller/FollowPath`, 10 Hz, desired linear speed 0.22 m/s. Both tests used Map-3 at 3x scene scale, one fixed box from the map manifest, and a disabled pedestrian. Unknown cells remained blocked (`allow_unknown=false`, `track_unknown_space=true`). The accepted static-box edge clearance was 0.61 m.
- Both traversals shared scene `reset10042_scene1369567760`, the same corrected Nav2-only map and saved local-costmap SHA-256 `b778701e0d86e1e3cd3808b1138a5d7e1e7778542fc72f72b3bcce06c79332f9`. The original candidate map YAML/PGM were not modified.

## Results

| Direction | Nav2 route length | Nav2 FollowPath | Proxy sim time | Final goal distance | Minimum fixed-box edge distance | Contact / collision | Map violation | Timeout |
|---|---:|---|---:|---:|---:|---|---|---|
| Forward | 10.962 m | succeeded (status 4) | 50.264 s | 0.241 m | 0.897 m | none | no | no |
| Reverse | 10.343 m | succeeded (status 4) | 50.040 s | 0.237 m | 0.804 m | none | no | no |

Both local traces remained in traversable known-free map cells and within the proxy/map checks. The minimum reported edge distances exceeded the tested 0.61 m requirement. This is a positive, closed-loop result for these two traversals, not proof that all fixed obstacles or map boundaries are safe.

## Preserved run records and interpretation

- Forward accepted local run: `20261002_nav2_rpp_mujoco_fixed_gate_seed10042_rerun.json`; matching Jetson action result: `20261002_nav2_follow_result_seed10042_forward.json`.
- Reverse raw local run: `20261002_nav2_rpp_mujoco_reverse_fixed_gate_seed10042.json`; matching Jetson action result: `20261002_nav2_follow_result_seed10042_reverse.json`; separate reconciliation: `20261002_nav2_reverse_gate_adjudication.json`.
- The reverse local file is intentionally **not rewritten**: it says `fail` because the runner missed the action terminal transition in a race, although the same record contains result code 4, goal distance 0.2371 m, and no safety violations. The separately saved Nav2 action result for the matching scene/path says `succeeded` (status 4); runtime bridge health also showed last status 4. Under the explicit pass rule in the adjudication, the run is adjudicated pass while its original raw label remains visible.
- An earlier forward attempt is also preserved: `20261002_nav2_rpp_mujoco_fixed_gate_seed10042.json` and `20261002_nav2_follow_result_seed10042_forward_initial_failure.json`. Its local harness stopped once it was within 0.30 m, before observing Nav2 success; the subsequent action result was aborted. It is not counted as a pass. The follow-up runner now waits for Nav2's terminal result.
- Route geometry evidence: `20261002_nav2_path_seed10042_controller_loop.json` and `20261002_nav2_path_seed10042_reverse_controller_loop.json`. Saved controller local costmap: `local_costmap_seed10042_controller_static.json` and `.bin`.

## Acceptance boundary and next gate

This closes the initial controller-in-loop check for **one scene, two directions, one fixed obstacle**. It does not yet establish a general boundary-avoidance guarantee: these trajectories did not constitute a deliberate boundary-approach suite. Mirrored obstacle/route placements and additional fixed-object/map-edge cases still need to pass with zero contact, zero map violation, and no timeout before enabling active people. Dynamic pedestrian behavior, PPO quality, locomotion, and real-robot safety remain untested here. No PPO training was started in this work.

Rollback: this was an isolated simulation-only run; no production map/configuration, driver, safety behavior, or hardware state was changed. The experiment can be discarded without reverting the source candidate map or formal Nav2 configuration.
