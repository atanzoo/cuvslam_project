# Map-3 scenario 10042: real Nav2 map/path gate and partial proxy rollout

Date: 2026-10-01 (Asia/Taipei)  
Status: **Nav2 map/path geometry PASS for this one scene; path-fed MuJoCo rollout FAIL (timeout). No training started.**

## Boundary and runtime

- Jetson host: Ubuntu 20.04, ROS 2 Foxy, Nav2 planner 0.4.7. New isolated directory `/home/tseng/jetson_slam_ws/simulation/path_planning/nav2_map3_20261001`; `ROS_DOMAIN_ID=43`, `ROS_LOCALHOST_ONLY=1`. Only map server, planner server, lifecycle manager, and static map-to-start TF were launched. Both lifecycle nodes reached `active`; there was no Nav2 controller, `/cmd_vel`, motor use, production-workspace change, or real-robot deployment. The exact planner-only launch session was stopped after evidence capture; process check found no remaining launch/planner processes.
- Local Mac: project `.venv`, D1 Max **kinematic proxy** in MuJoCo. The true Nav2 path was loaded into the existing local `Nav2GlobalPath` consumer for one 70 s simulation rollout, but Nav2 was **not** running as a controller during this rollout. This is a partial integration check, not true end-to-end Nav2 closed-loop acceptance.
- Jetson host wall clock reports 1970. Runtime ROS timestamps cannot be used as 2026 evidence until clock/sim-time handling is validated. The date above is the Mac/client date.
- Scenario identity `reset10042_scene1369567760`: reset seed 10042, curriculum 0, 3× map, 10 m requested route, fixed box center `(-3.145, 2.3179942314911184)` m and 0.25 m half-extents. The pedestrian was **not** baked into the map. In the path-fed fixed-object gate only, the physical fixed box remained active and the pedestrian was disabled.

## Safety-significant map-server discovery and correction

The original candidate-map PGM has 256,518 gray-205 cells after 3× replication. Its source YAML has `free_thresh: 0.25`. The local candidate-map reader treats pixel 205 as Unknown explicitly, but the real Foxy `map_server` in `trinary` mode treated gray 205 as **Free** with that threshold: the first isolated map-server readback was `{-1: 0, 0: 512042, 100: 39586}`. The costmap likewise had zero 255 Unknown cells. Therefore the initial Nav2 path/costmap run was **rejected**, even though it reported a path and a fixed-box lethal cell. Do not use that initial run as safety evidence.

The exporter was changed to make a separate Nav2-only YAML with `free_thresh: 0.19` and validate the Nav2 trinary class of every unique PGM value. The PGM pixels, resolution, origin, physical box and original source files were unchanged. The corrected map is `output/nav2_map3_static_seed10042_nav2trinary_20261001/`; the original failed output was retained. Local and Jetson copies matched SHA-256:

| Artifact | SHA-256 |
|---|---|
| Original candidate YAML | `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d` |
| Original candidate PGM | `41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74` |
| Corrected Nav2-only YAML | `ff5cd429ccb86783e6e39ce6017e2c40f1ec71a4e01c364e8b9799ad2f63a33e` |
| Corrected Nav2-only PGM | `4acb7b47b6c0315da74e18e7c0347bd0603627cc5389790b8668ecdce2aab921` |
| Corrected manifest | `f76335177619beb59f1de63d8442fb3b427a6c29d0c4889df5e5e77b63b5fb63` |

On the corrected map, the real `/map_server/map` response was exactly `{-1: 256518, 0: 255524, 100: 39586}` across 597×924 cells at 0.05 m/cell. The global costmap had 208,099 cells still marked 255 Unknown; inflation marked some other Unknown cells 253, also non-traversable. A full cell-by-cell bottom-up costmap check confirmed **all 256,518 source Unknown and all 39,586 source Occupied cells have cost ≥253**. The fixed-box center cost was 254, the start cost was 0. `allow_unknown=false`, `track_unknown_space=true`, and `robot_radius=0.61 m` were inspected on the live nodes.

## One-scene path result and limit

Real `compute_path_to_pose` action returned 437 points and a 10.96156 m path. Against the corrected export, 654 geometric samples were traversable, minimum center-to-fixed-box-edge distance was 0.8690 m versus 0.61 m required, and start/goal errors were 0.0252/0 m. Separately, 1,089 samples against the **actual saved Nav2 costmap** stayed below 253 and inside bounds. This is a one-scene planner/path geometry PASS only. The action client emitted a Foxy `InvalidHandle` destructor warning after successfully saving the path; its local source now explicitly destroys the client before node shutdown, but that revised source has not yet been rerun on Jetson.

The path, costmap summary/raw bytes, and local rollout are retained as:

- `nav2_path_seed10042_nav2trinary.json` — SHA-256 `d2d6a52140b628907348f8f8cdab7c29aaf4a222058f9d3fdea331392488d2e8`.
- `costmap_seed10042_nav2trinary.json` and `.bin` — raw bytes SHA-256 `b778701e0d86e1e3cd3808b1138a5d7e1e7778542fc72f72b3bcce06c79332f9`.
- `20261001_nav2_path_fed_proxy_gate_seed10042.json` — local fixed-only 70 s rollout, with 1 Hz trace.

## Partial closed-loop result: FAIL

With action 0 requested on every local decision step and the true Nav2 path fed into the existing MuJoCo/MPPI/SDK chain, the fixed box remained physical. The run finished at 70.0 s after 730 decisions: **0 success, 0 physical collision, 0 map violations, 1 timeout**. Minimum simulated robot-center-to-fixed-box-edge distance was 0.7004 m, above the 0.61 m tested radius-plus-margin. The sampled goal distance was about 0.58 m at 58 s, then rose to 2.0664 m at timeout while the robot traveled past the goal. Executed mode counts were AVOID 613, CRUISE 117. The saved first-version terminal contact list includes floor-to-obstacle contacts and must **not** be interpreted as robot collisions; the actual environment `physical_contact` and `collision` flags were false. The gate script was corrected to filter floor contacts in future evidence, without editing this original run.

This is direct evidence that a collision-free Nav2 global path is **not sufficient** for the current local control chain to complete this scene. It does not isolate whether the failure is path-reference terminal behavior, persistent AVOID mode, local MPPI, or goal approach. It also does not validate a Nav2 controller, mirror cases, active pedestrians, or real-robot navigation.

### Bounded controller diagnosis, also FAIL

A second unchanged-baseline replay (`20261001_nav2_path_fed_proxy_gate_seed10042_diagnostic_v2.json`) reproduced the 70 s timeout, with extra 1 Hz control trace. Around 58–69 s, AVOID was persistent, path-heading error was about 1.5 rad, and the recorded SDK yaw velocity was 0; lateral error grew while goal distance increased. This supports a terminal path-following/control mismatch, but is not a complete causal isolation.

One **in-memory experimental** comparison used the same existing path-heading command in AVOID mode as in CRUISE, without editing the environment/controller source or training configuration. Its separate artifact is `20261001_nav2_path_fed_proxy_gate_seed10042_avoid_heading_probe.json`. It **collided with the fixed object at 46.288 s**, not at the goal: `moving_obstacle_geom` contacted `FBL_FOOT_LINK_collision`, contact penetration about `-0.00054 m`, and minimum center-to-box-edge distance was 0.4532 m (<0.61 m tested radius-plus-margin). No map violation was recorded. This falsifies the simple “add yaw feedback in AVOID” fix for this scene. Do not promote this experimental command to the main controller or training setup.

## Verification and next gate

- Focused exporter/path/costmap unit tests: 9 PASS, including rejection of Unknown converted to free and paths through lethal costmap cells. Python compile passed.
- Source candidate YAML/PGM hashes remained unchanged. Remote evidence copies matched local hashes. No previous map, config, checkpoint, logs, or stopped training output was deleted or overwritten.
- **Do not start 25k PPO.** A controller change now needs a specific safety design/review: coordinate path following with costmap/footprint clearance rather than adding yaw feedback alone, or complete the approved isolated Nav2 controller bridge. Re-run fixed-only scenes and mirrors with goal arrival, zero fixed contact, zero map violation, zero timeout. Only after that run active-pedestrian families and train under the documented gate.
