# Real D435i + RPLIDAR A2M12 Integration Preparation

Started: 2026-08-31 (Asia/Taipei)  
Last update: 2026-09-03 (Asia/Taipei)  
Status: GUI mapping runtime integrated and stationary data path passed; calibrated extrinsics, queue behavior, and motion acceptance pending  
Change class: C2 + C4; new sensor integration boundary  
Scope: real D435i/cuVSLAM odometry and 2D RPLIDAR mapping only

## Objective

Prepare a separate real-robot launch that combines the accepted D435i/cuVSLAM
odometry line with an RPLIDAR A2M12 and `slam_toolbox`:

```text
D435i/cuVSLAM odometry-only -> odom -> base_link
static camera extrinsic     -> base_link -> camera_link
RPLIDAR A2M12 /scan         -> rplidar_link
static LiDAR extrinsic      -> base_link -> rplidar_link
slam_toolbox                -> map -> odom
```

The A2M12 path is a 2D mapping experiment. It does not replace the planned 3D
LiDAR phase and does not add Nav2 or robot velocity control.

## Non-goals and protected assets

- No Nav2, `/cmd_vel`, autonomous motion, or safety-controller changes.
- No change to the existing R2 runner's default behavior.
- No modification of `/Applications/slam_v2` or the external LiDAR/Nav2 line.
- No use of simulation extrinsics as real calibration.
- No deletion of bags, maps, calibration, logs, or previous evidence.
- The existing `real_d435i_rviz` container remains a visualization-only asset.

## Read-only Jetson inventory

Target: `tseng@192.168.55.1`, host reported `ubuntu`.

Observed USB devices:

```text
Intel D435i:       8086:0b3a
USB serial bridge: Silicon Labs CP210x / CP2102
serial device:     /dev/ttyUSB0
stable path:       /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_538fdae3c9e4c947b71ec98a5cebbe00-if00-port0
```

The active `real_d435i_rviz` container has no Docker device mapping. It runs
RViz and bridge processes only, so it is not reused for the integrated camera
and LiDAR runtime.

Packages found inside the existing Isaac ROS Humble container:

```text
slam_toolbox
laser_geometry
```

Not found:

```text
rplidar_ros
sllidar_ros2
```

The official Slamtec ROS 2 repository provides an A2M12 launch entry and its
profile uses the model-specific serial rate. The deployed profile records
`serial_baudrate=256000`, `angle_compensate=true`, `scan_mode=Sensitivity`,
and `frame_id=rplidar_link`. The `sllidar_ros2` package was cloned from the
official repository and compiled in the Jetson Isaac ROS workspace; the live
container exposes `sllidar_node` and the A2M12 scan path.

## Local changes staged

- `real_robot/cuvslam/config/rplidar/a2m12.yaml`
  - A2M12 serial profile and the observed stable udev path.
- `real_robot/cuvslam/config/real_d435i_rplidar_slam_toolbox.yaml`
  - `odom_frame=odom`, `base_frame=base_link`, `scan_topic=/scan`, mapping mode.
- `real_robot/cuvslam/deployment/isaac_ros/isaac_ros_visual_slam_d435i_rplidar.launch.py`
  - starts static TF, A2M12 driver, cuVSLAM odometry-only, and async
    `slam_toolbox` after a short TF startup delay. Its D435i `initial_reset`
    default is false to match the accepted R2 no-reset runtime path.
- `real_robot/cuvslam/deployment/isaac_ros/isaac_ros_visual_slam_d435i_real.launch.py`
  - adds launch overrides for `base_frame`, `input_base_frame`, and
    `publish_map_to_odom_tf`; historical defaults remain unchanged.
- `real_robot/cuvslam/tools/run_real_d435i_rplidar.sh`
  - separate container lifecycle and compressed evidence recording helper.
- `real_robot/cuvslam/README.md`, `docs/README.md`, and `PROJECT_HANDOFF.md`
  - record the boundary, missing dependency, and acceptance limitation.

## Temporary extrinsics

The integrated launch exposes these replaceable parameters:

```text
camera_x camera_y camera_z camera_yaw
lidar_x  lidar_y  lidar_z  lidar_yaw
```

All defaults are `0.0`. This is an explicit placeholder for the first
stationary wiring check, not a calibration result. The camera transform is
also required because the current historical real launch used `camera_link`
as its cuVSLAM base output, while the integrated contract uses `base_link`.

## Ownership and failure boundaries

- cuVSLAM must run with mapping/localization disabled and
  `publish_map_to_odom_tf=false`.
- `slam_toolbox` is the only integrated publisher of `map -> odom`.
- Static publishers own only the D435i and A2M12 sensor edges.
- The A2M12 driver owns `/scan`, not TF for the robot body.
- The R2 and R1.5 runners are mutually excluded by the new integration script
  to avoid opening the D435i from two containers.

Primary failure modes are missing `sllidar_ros2`, wrong serial rate or device
permissions, incorrect placeholder extrinsics, a disconnected camera TF tree,
and duplicate dynamic `map -> odom` publishers.

## Verification ladder

Completed locally:

- Python syntax compilation of both launch files.
- Bash syntax check of the new runner.
- `git diff --check`.

Completed on Jetson, stationary only:

- Official `sllidar_ros2` package build for the target container.
- A2M12 detection and scan output: serial S/N
  `56F1EDF9C7E29BD1A7E39EF2C622431B`, firmware `1.32`, hardware revision `6`,
  health `OK`, Sensitivity mode, approximately `12.2 Hz`,
  `frame_id=rplidar_link`, range limits `0.05--16.0 m`.
- D435i camera stream and CameraInfo sample at `640x360`, `mono8`, with the
  RealSense publisher active.
- cuVSLAM odometry sample with `frame_id=odom`,
  `child_frame_id=base_link`, approximately `27.2 Hz`, and
  `/visual_slam/status` reporting `vo_state=1`.
- TF ownership check: cuVSLAM publishes `odom -> base_link`, the static
  publishers provide the sensor edges, and slam_toolbox publishes
  `map -> odom`.
- slam_toolbox dynamic-map query: `195x117` at `0.05 m` resolution, with
  `239` occupied, `5465` free, and `17111` unknown cells. This confirms that
  LiDAR data reached the mapper; it is not a map-quality result.

Observed warnings and limitations:

- The first integrated start inherited `initial_reset=true`; D435i stopped at
  `Resetting device...` while A2M12 was already healthy. The integrated launch
  was changed to default `initial_reset=false`, and the second start brought up
  the D435i publisher and cuVSLAM odometry.
- RealSense emitted `Motion Module failure`, UVC watchdog, temporary USB
  resource, and missing IMU-calibration warnings. The data path worked for this
  stationary check, but IMU robustness is not accepted by this record.
- During cuVSLAM GPU/startup warm-up, slam_toolbox dropped queued scans while
  the odom/TF chain was not yet available. The final stationary integration
  check still observed 3 queue-drop messages in the most recent 60-second log
  window even though `/map` and map-frame projection were available. Treat
  queue behavior as a follow-up issue, not as a map-quality pass.

Not yet run:

- calibrated extrinsic replacement and numerical calibration audit;
- controlled motion, vibration, tracking-loss, restart, or map-quality tests;
- RViz visual inspection, map save/reload, Nav2, `/cmd_vel`, or any robot motion.

## GUI live mapping observation update (2026-09-03)

The macOS R2 Web GUI and its remote live telemetry collector were extended for
the 2D LiDAR observation stage. The collector now subscribes to `/map` with a
transient-local map QoS, `/scan` with a LiDAR QoS, and TF. It emits a compressed
occupancy grid only when the map changes, continuous scan points transformed
into the map frame, and a map-frame projection of the retained cuVSLAM odom
path. The GUI renders these as separate layers while preserving the original
odom-frame path view and R2 session/trace records.

The GUI now routes `mapping（建圖）` to the separate
`real_d435i_rplidar` runner and routes `odom` to the existing R2 runner. The
mapping runner exposes matching telemetry, image-quality, recording, and
cleanup entry points, and the light bag includes `/map`, `/scan`, TF, IMU,
cuVSLAM odometry, and status. This does not change TF ownership, replace the
zero extrinsics, or add Nav2/`/cmd_vel`.

Local verification completed:

- Python syntax compilation for the GUI, collector, and integration contract test.
- Static integration contract passed with
  `REAL_D435I_RPLIDAR_CONTRACT_PASS`.
- GUI runtime routing smoke passed with `GUI_RUNTIME_ROUTE_PASS` for both
  mapping and odom selections.
- Synthetic GUI map/scan/odom parsing smoke passed with
  `GUI_MAPPING_SMOKE_PASS`.
- `git diff --check` passed.

Jetson deployment completed on 2026-09-03:

- Updated `/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/collect_real_d435i_odom_live.py`.
- Remote deployed SHA-256:
  `75a9a1184a530cc072b1ff946d998d06b8ecbb47dc49c73a541b138687169f45`.
- Remote `python3 -m py_compile` passed and executable mode `755` was preserved.
- Previous remote file was preserved at
  `/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/collect_real_d435i_odom_live.py.bak_20260903_gui_map`
  with SHA-256
  `5ba6ea9f2b0657e98806e06c268ca8223535b9a318ae3d4aaceae4d2731b7ea0`.
- No Jetson container was started, stopped, or restarted during this sync.

The mapping runner was then synchronized and checked:

- Updated `/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh`.
- Remote deployed SHA-256:
  `a39e2375a92613992c0d0a9e9d59d97452fda671c68d08dcd0fa1237aea38ab1`.
- Remote `bash -n` passed and executable mode `755` was preserved.
- Previous remote runner was preserved at
  `/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh.bak_20260903_gui_mapping`
  with SHA-256
  `54f582590f704fd9754679e9f1f40df38edc15cd5393cf22197eea69cdb8eed1`.

A follow-up stop-flow clarification was synchronized after the stationary
check:

- Current runner SHA-256:
  `8d12796560fad84546fc97e508579e6e62d26374d7f863768a48fe4488f3cb66`.
- The previous deployed runner was preserved at
  `/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh.bak_20260903_stop_motor_result`
  with SHA-256
  `24572cdb6c6f68ff530f49a523a05a366b8031c308e9f6778c9a6f2dfab1dfc0`.
- The runner's stop output now explicitly records that D435i, cuVSLAM, the
  in-container A2M12 `sllidar_node`, and slam_toolbox are stopped together;
  the separate RViz observer is left running.
- The stop sequence first calls the driver-provided `/stop_motor` service and
  then stops/removes the container. If the service is unavailable, the runner
  still performs force cleanup and reports that fallback in its output.
- A live stationary stop test returned
  `lidar_stop=stop_motor_service_called`; afterward no process occupied
  `/dev/ttyUSB0`, no `real_d435i_rplidar` container remained, and only the
  separate RViz observer was still running.
- The stop test also confirmed that a printed `Empty_Response` is treated as
  success even if the ROS 2 CLI reaches its timeout while exiting.

## Integrated GUI mapping stationary check (2026-09-03)

The Jetson runner was started through the canonical path while the D435i and
A2M12 remained stationary. The runtime started `sllidar_node`, RealSense,
cuVSLAM, and `async_slam_toolbox_node`. A2M12 reported S/N
`56F1EDF9C7E29BD1A7E39EF2C622431B`, firmware `1.32`, hardware revision `6`,
health `OK`, Sensitivity mode, and approximately `10 Hz` device scan rate.
D435i serial `261222077990` was detected on USB 3.2, and cuVSLAM reported
version `11.4`.

The stationary ROS checks recorded approximately `12.0 Hz` on `/scan` and
`30.0 Hz` on `/visual_slam/tracking/odometry`. `/map` produced a `199x116`
occupancy grid at `0.05 m` resolution. The synced collector emitted map,
scan, and odom streams; the final snapshot contained `source_points=1800`,
`display_points=401`, `samples=199`, `vo_state=1`, and `map_path_points=199`.
The collector's map-frame projection therefore reached the GUI data contract.
Direct tf2 lookup also returned `map -> odom` and
`base_link -> rplidar_link` transforms; the current values are zero
placeholders and are not calibration evidence.

A light recording smoke test succeeded and was stopped cleanly. The preserved
remote evidence is:

```text
/home/tseng/isaac_ros_data/real_robot/cuvslam/experiments/real_d435i_rplidar/smoke_20260903_gui_mapping/
  rosbag/metadata.yaml       5167 bytes
  rosbag/rosbag_0.db3.zstd   387122 bytes
  rosbag_record.log          1967 bytes
```

The mapping container was stopped after the check; this also stopped the
in-container A2M12 `sllidar_node`, D435i, cuVSLAM, and slam_toolbox. The
pre-existing `real_d435i_rviz` observation container remained running. No robot motion,
Nav2, `/cmd_vel`, map save/reload, calibrated extrinsics, or map-quality test
was performed. The remaining queue-drop warning is recorded above.

## Stage 1 — low-light runtime and evidence plumbing (2026-09-03)

Stage 1 is implemented in the local repository and has been synchronized to
the Jetson runtime paths. The macOS GUI now forwards the
selected mode, IR profile, lighting profile, and diagnostics mode to both the
R2 odometry runner and the separate D435i + A2M12 mapping runner. The new
`low_light_motion` profile is explicitly experimental: it keeps the IR
projector enabled and requests manual `10,000 us` exposure with gain `24`.
The existing `official`, `baseline`, `robust`, and `high_contrast` profiles are
otherwise unchanged.

The image-quality collector now records D435i per-frame metadata for both IR
streams, including actual exposure, gain, automatic-exposure state, actual
frame rate, laser power, and emitter mode. Full mapping and R2 recordings also
retain the metadata and cuVSLAM observation/pose-graph topics so that a later
low-light A/B can connect image conditions to tracking behavior. The GUI
assessment surfaces collector flags and adds an explicit geometry gate for the
existing turn-translation candidate instead of allowing a healthy stream/rate
score to hide that warning. The live quality panel also displays the measured
exposure and gain for both IR streams when metadata is available.

The GUI work-record panel now has a separate `複製工作記錄（過程）` action.
It copies the chronological process log only, including startup parameters,
connection events, warnings, stop/cleanup output, and errors; the conclusion
summary remains a separate copy action. Each run now adds a readable Requested
runtime block at startup, an Actual D435i metadata block when measurements
arrive, quality/geometry state changes, and a final quality/geometry block
before the session is saved. This preserves the distinction between requested
camera values and values actually reported by the device.

Scope remains limited to runtime parameters, diagnostics, and evidence
collection. This stage does not add LiDAR scan-matching odometry, EKF fusion,
Nav2, `/cmd_vel`, a new TF owner, or calibrated extrinsics. Jetson
synchronization completed after dated backups were created; the runner,
collector, and both copies of the integrated launch now match the local
SHA-256 values. No container was started or stopped during this sync, and no
low-light motion acceptance test has been run. The `low_light_motion` values
must be checked against the target D435i's actual metadata before being treated
as a usable profile.

## 2026-09-11 — LiDAR translation parameter plumbing

The first local implementation step for the fixed A2M12 mount is complete.
`CUVSLAM_LIDAR_X`, `CUVSLAM_LIDAR_Y`, and `CUVSLAM_LIDAR_Z` now pass through
the local environment, GUI requested-runtime record, chronological work log,
mapping runner, and integrated launch. The runner validates the values as
numeric inputs and preserves the existing positional interface by appending
them after the emitter override. The runtime trace records all four static-TF
inputs (`x/y/z/yaw`).

The current local values are `x=0.0`, `y=-0.10`, `z=0.0`, and `yaw=2.334` rad.
The user supplied the approximate 10 cm horizontal offset with the LiDAR on
the camera's right; it is represented as negative `base_link` y. No physical
hardware was moved, and this is still a hardware test candidate rather than
verified extrinsic calibration evidence.

Local `py_compile`, runner `bash -n`, the real D435i + RPLIDAR contract test,
and whitespace checks passed. The first Jetson connection attempt was blocked
by the USB network, then recovered on 2026-09-11. The validated runner was
synchronized to the canonical remote path; local and remote SHA-256 both equal
`9f65508d135522667c0d7d8b672a976817279e226ecf9b05aeed89a4baa2f86c`.
The dated rollback copy is
`/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh.before_lidar_y_20260911_094344`.
No container or ROS/hardware run was started. The next gate is GUI startup
with explicit `y=-0.10 m`, followed by stationary, straight, and labelled-turn
checks. LiDAR odometry, EKF, Nav2, TF ownership, and map-quality acceptance
remain unchanged.

### 2026-09-03 official-profile run review

The run saved at
`real_robot/cuvslam/evidence/logs/real_d435i_quality/20260903_143209_r2_session.json`
used `official`, `mapping`, `640x360x30`, and `motion=other`. Its transport
and stream health were good (approximately 29.7 Hz image streams, 30 Hz
odom, 0 us stereo-sync P95, and 100 percent raw VO-valid samples), but the
trajectory evidence was not accepted: the process log recorded transient
clipped/stereo-desync flags, turn-jitter REVIEW, and turn-translation FAIL
observations up to `0.696 m` XY at `89.8 deg`. The final approximately
`0.265 m` XY closure residual also exceeded the `0.20 m` review reference. The
motion label was `other`, so this is diagnostic evidence rather than a formal
pure-turn or closed-loop acceptance test.

The metadata shows that `official` requested automatic exposure with an
8500 us target and gain 16; the device reported automatic exposure enabled,
gain 16, emitter enabled, and actual exposure values varying roughly from
3133 to 4720 us during this run. This confirms that the requested value and
the device-reported value must remain separate in the research record.

The historical session and trace are preserved unchanged. After reviewing
this run, the GUI was updated to retain the union of observed image flags and
the worst turn-geometry gate across the entire session, so a later
`未偵測` final snapshot cannot erase an earlier FAIL/REVIEW.

Local verification for Stage 1:

- Bash syntax checks for both real runners.
- Python syntax compilation for the updated collector, GUI, and contract test.
- Static `REAL_D435I_RPLIDAR_CONTRACT_PASS` after extending the contract to
  cover parameter forwarding, metadata topics, and the new quality gates.
- `git diff --check`.

Remote verification completed on Jetson:

- Bash syntax checks passed for both runners.
- Python `py_compile` passed for the image-quality collector and both launch
  copies.
- Deployed SHA-256 values:
  - R2 runner `5096575ffa797510178cb2a748bb5ffd65537b05a03f05a70733cc8a1a65820a`;
  - mapping runner `48570674c3911bc1439f1fab76e9c63829e4147d6845daaea105a0273074b966`;
  - image-quality collector `5a2a99105e059353d7c61d4a643abf463688a8a8635244d8af8e8bc078f470cf`;
  - both integrated launch copies `0a5b000be0b9426f178f53c833cc4005916ccccb5c11d1d6e089389a4ed8ea79`.
- Dated rollback copies use the suffix `.bak_20260903_stage1` beside each
  deployed file.

Rollback is limited to the local changed files: select an existing lighting
profile, use the previous runner/collector backup on Jetson if applicable, or
revert this Stage 1 patch. Existing odometry/mapping ownership and historical
evidence remain unchanged.

Remaining hardware sequence:

1. Replace the zero placeholders with measured D435i and A2M12 extrinsics.
2. Re-run the stationary TF and frame audit with the calibrated values.
3. Record a short compressed bag containing `/scan`, `/tf`, `/tf_static`,
   `/camera/imu`, cuVSLAM odometry, and status.
4. Only after the calibrated stationary gate passes, request a separate
   controlled motion test.

## Rollback

Stop and remove only the `real_d435i_rplidar` container. Restore the mapping
runner from
`/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh.bak_20260903_gui_mapping`
if the new runner behavior must be rolled back. Remove or revert the new
A2M12 launch/config/runner files if the staged design is rejected. The
existing R2/R1.5 launch files, RViz container, historical evidence, and
simulation workstream remain available without this integration.
