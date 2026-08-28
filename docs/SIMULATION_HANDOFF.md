# Simulation Handoff

Updated: 2026-07-31 (Asia/Taipei)

This document is the entry point for simulation and replay work. It is
separate from the real-robot deployment procedure and from the existing LiDAR
ICP/Nav2 project.

## Current Baseline

- Target computer: Jetson AGX Orin Developer Kit
- JetPack: 5.1.3 / Jetson Linux R35.5.0
- Host ROS 2: Foxy
- Isaac ROS container ROS 2: Humble
- Isaac ROS image: `isaac_ros_dev-aarch64:fixed`
- Isaac ROS workspace: `/home/tseng/isaac_ros_ws`
- Data directory: `/home/tseng/isaac_ros_data`
- Isaac ROS domain: `42`
- Verified build: 36 packages, including `isaac_ros_visual_slam`

The Isaac ROS container has passed CUDA, package, and USB-bus preflight. A
RealSense device and firmware have not yet been verified on the current
session.

The Mac GUI start path was repaired on 2026-07-31. It now verifies and uses
the dense v3 SDF under the installed package `share/slam_gazebo` path, starts
stereo-only cuVSLAM with mapping visualization, and leaves the large native
Pose_V bridge disabled. The native bridge is enabled only by route tests that
use the compact relay. A live verification reached `vo_state=1`. See
`reports/gui_start_simulation_fix_20260731.md`.

## 2026-07-31 In-Place Turn Pose Gate

The 90-degree stereo-only turn gate now passes in the dense-turn v3 world.
Wheel odometry is absent from the bridge, bag, controller, cuVSLAM inputs, and
evaluation. D435i gyro integration controls the turn; Gazebo native dynamic
pose is the only physical truth.

| Cold start | Yaw error | cuVSLAM false translation | Tracking lost |
|---|---:|---:|---:|
| v3 run 1 | -0.75 deg | 0.0030 m | 0 |
| v3 run 2 | +0.08 deg | 0.0167 m | 0 |

Both pass the 5-degree yaw and 0.05 m false-translation limits. Use:

```text
cuvslam_mapping_observable_640x360_range2p5_dense_turn.sdf
stereo-only, mapping enabled, 20 deg/s turn, camera_x=0.19 m
```

The original scene retained a 10--14 cm false-translation floor even when
speed fixed yaw. VIO, camera-at-origin, VO-only, and short-render-range tests
did not fix it. Continuous asymmetric near-field geometry across the entire
turn did. Proceed to `1 m -> 90 deg -> 0.5 m`; do not restore wheel odometry
for linear control. See
`reports/cuvslam_in_place_turn_pose_fix_20260731.md`.

The first `1 m -> 90 deg -> 0.5 m` mapping run is complete but failed the
route pose gate. Native/IMU control was correct and all 852 status samples
remained at `vo_state=1`, but cuVSLAM under-estimated the turn by 11.59
degrees. The dense feature arc fixed rotation only at the initial pose; this
route turns approximately one metre farther forward.

Current observations were 99.3 percent within 0.25 m of SDF surfaces. The
final 4,243 landmarks also scored 99.5 percent within 0.25 m, but this nearest
surface score masks the wrong route orientation and 22 extreme outliers up to
54.4 m. The map is generated but not accepted. Extend dense, asymmetric
near-field geometry around `(-0.8, -1.8)` and repeat. See
`reports/cuvslam_initial_dense_v3_mapping_20260731.md`.

## 2026-07-30 Current Gate

Historical note: this gate is superseded by the 2026-07-31 observability gate
below. It is retained to preserve the investigation sequence.

Do not continue square mapping, turning, map-quality tuning, IMU fusion, or
Nav2 integration yet.

The installed cuVSLAM runtime passed the NVIDIA release-2.1 official stereo
control: 725 input left images produced 725 odometry and status messages, all
with `vo_state=1`. This isolates the current failure to the Gazebo-side input
path rather than proving a cuVSLAM algorithm defect.

A controlled native-truth run moved the level chassis `1.0176 m`. cuVSLAM
reported only `0.1914 m` and `5.05 deg` false rotation. NVIDIA's debug dump
then proved that 501 consecutive left images and 501 consecutive right images
were byte-identical during ongoing physical model motion, despite advancing
timestamps.

The next gate is therefore:

```text
Gazebo Transport image
  -> ROS bridge image
  -> NVIDIA debug-dump image
```

Capture and hash the same straight-motion interval at all three boundaries.
Locate the first stale-pixel boundary before changing code. TF, CameraInfo,
baseline, output frame ownership, and cuVSLAM parameters are locked for this
experiment.

Primary evidence:

- `reports/nvidia_reference_and_gazebo_1m_debug_20260730.md`
- `docs/CUVSLAM_NVIDIA_REFERENCE_DEBUG_DESIGN.md`
- `docs/NEXT_THREAD_MAPPING_TEST_PROMPT.md`

The user-provided power bank passed a short CPU and representative
Gazebo/cuVSLAM load test in `MODE_30W`. See
`reports/jetson_powerbank_stress_test_20260730.md`. This does not yet qualify
battery runtime or low-battery shutdown behavior.

At handoff completion, all simulation workloads were stopped and the Jetson was
shut down normally. A follow-up SSH probe timed out, confirming that the Jetson
was offline.

## 2026-07-31 1 m Observability Gate

The stale-pixel hypothesis was excluded in a reliable cold-start baseline:
every in-motion bridged and NVIDIA debug stereo frame was unique, but cuVSLAM
reported only `0.0749 m` for `1.0116 m` native truth.

The independent near-field observable-world A/B preserved camera geometry,
TF, CameraInfo, timing, IMU setting, and cuVSLAM parameters. It reported
`1.0232 m` for `1.0128 m` native truth with `0.98 deg` rotation error and
`vo_state=1` for all 486 samples.

Use:

```text
cuvslam_mapping_observable.sdf
```

for the next simulation repeatability and mapping gates. Preserve
`cuvslam_mapping_simple.sdf` as the sparse-scene negative control. Do not
generalize this simulation-scene fix to real-camera accuracy.

The observable world was then expanded to cover all four square-route legs
with 30 visual-only near-field features. Two additional cold-start trials
passed:

| Trial | Native truth | cuVSLAM | Rotation |
|---|---:|---:|---:|
| Repeat 1 | 1.0176 m | 1.0189 m | 0.78 deg |
| Repeat 2 | 1.0152 m | 0.9708 m | 1.32 deg |

Every in-motion image was unique, stereo timestamps matched exactly, and all
status samples remained `vo_state=1`. The straight-motion repeatability gate
is complete. Begin mapping with a straight segment, then one controlled
90-degree turn, and only then the full square. See
`reports/cuvslam_observable_world_repeatability_20260731.md`.

## 2026-07-31 First Straight Mapping Geometry Gate

The first 1 m observable-world mapping run passed trajectory accuracy but
failed sparse-map geometry:

- truth `1.0140 m`, cuVSLAM `0.9548 m`, rotation `1.46 deg`;
- fixed-scale translation ATE P95 `0.0746 m`;
- 581 / 581 status samples at `vo_state=1`;
- 379 final landmarks;
- 35.9 percent within 0.10 m of an SDF surface;
- 50.1 percent within 0.25 m;
- surface-distance P90 `1.7618 m`, maximum `5.0750 m`.

Do not enter the 90-degree turn gate yet. Repeat the same cold-start straight
mapping once, then inspect landmark confidence and stereo triangulation if the
heavy-tailed geometry error repeats. See
`reports/cuvslam_first_observable_straight_mapping_20260731.md`.

The repeat confirmed the failure: only 47.9 percent of 378 landmarks were
within 0.25 m of an SDF surface, P90 was `1.8473 m`, and maximum error was
`6.7848 m`. Approximately 45 percent of final landmarks matched the first run
within 1 mm, showing a predominantly systematic map structure.

A visualization-only diagnostic exported cuVSLAM's internal `LL_MAP` weight.
Every point, including severe outliers, had weight exactly `1.0`. The patch
was reversed and the standard NVIDIA PointCloud schema was restored. The next
gate is current observation cloud versus accumulated map cloud plus stereo
disparity; turning remains blocked. See
`reports/cuvslam_landmark_repeatability_and_weight_20260731.md`.

## 2026-07-31 Triangulation And Out-And-Back Gate

The minimum current-observation test confirmed that geometry fails before map
accumulation. At `424x240`, points beyond 2.5 m had only about 2–3 pixels of
expected disparity and geometry quality collapsed. The declared physical
cloud frame was correct; an optical-frame counterfactual was much worse.

A controlled `640x360 @ 30 Hz` plus `2.5 m` stereo rendering-range variant
passed the provisional minimum observation gate with 87.2 percent of points
within 0.25 m of SDF surfaces, compared with 43.8 percent originally.

The subsequent 1 m straight-out/reverse-back run failed the complete gate:

- truth closure `0.0150 m`;
- cuVSLAM odometry closure `0.1834 m` and `6.07 deg`;
- translation ATE P95 `0.2045 m`;
- current observations 84.4 percent within 0.25 m;
- final `LL_MAP` landmarks 78.4 percent within 0.25 m, P90 `0.6129 m`;
- all 795 status samples remained at `vo_state=1`.

The backend map improved during re-observation but did not reject enough
wrong landmarks. Do not add a turn or begin the square route. See
`reports/cuvslam_triangulation_and_roundtrip_20260731.md`.

## 2026-07-31 Controlled Corner Exploration

An explicitly exploratory route tested the hypothesis that more time and a
new viewing direction could improve backend landmark maturation:

```text
1.0 m straight -> 91.89 deg left turn -> 0.5112 m straight -> 10 s settle
```

Tracking remained active for all 883 status samples, but geometry degraded:

- the in-place turn produced `0.4998 m` false translation;
- truth turn `88.75 deg`, cuVSLAM turn `64.64 deg`, error `24.23 deg`;
- translation ATE P95 `0.4736 m`, rotation ATE P95 `21.92 deg`;
- current observations remained at 83.2 percent within 0.25 m;
- `LL_MAP` peaked at 80.4 percent during the first straight, then fell to
  65.7 percent after the turn and second leg;
- final P90 was `0.7267 m`, with 1,005 / 2,934 landmarks over 0.25 m.

## 2026-07-31 D435i IMU And Native-Turn Correction

The controlled-corner reference was corrected after a D435i audit:

- `/ground_truth/odom` is DiffDrive wheel odometry and overstates turns under
  wheel slip;
- Gazebo native pose measured `79.28 deg` while wheel odometry approached
  `90 deg`;
- 200 Hz IMU integration measured `79.29 deg`, matching native pose within
  `0.01 deg`;
- the controller now uses timestamped IMU gyro integration to stop the turn;
- corrected validation reached `89.74 deg` by native pose and `89.76 deg` by
  independent bag IMU integration.

The current sensor cadence is 200 Hz IMU with 6--7 samples per image interval.
The camera is nominally 30 Hz but measures 28.57 Hz because of the 5 ms Gazebo
step.

An identical native-truth A/B did not accept IMU fusion:

| Metric | Stereo only | IMU fusion |
|---|---:|---:|
| Turn rotation residual | 11.28 deg | 12.72 deg |
| Turn false translation | 0.2738 m | 0.3251 m |
| Final landmarks within 0.25 m | 65.8% | 66.6% |
| Final surface-distance median | 0.0589 m | 0.0762 m |

Keep `enable_imu_fusion=false` by default. Use Gazebo native model pose for
trajectory acceptance and the IMU-gated controller for future physical
90-degree routes. See
`reports/cuvslam_d435i_imu_corner_audit_20260731.md`.

The longer-route convergence hypothesis is rejected for the current
configuration. Do not begin the full square. Isolate rotational image motion
and verify the camera-to-IMU contract before considering simulated IMU fusion.
See `reports/cuvslam_corner_exploration_20260731.md`.

## Verified Simulated Stereo Milestone

On 2026-07-27, the declared Gazebo D435i stereo source was connected to
`isaac_ros_visual_slam` in the isolated Humble container on ROS domain 43.
cuVSLAM 11.4 produced `vo_state=1`, odometry, VO pose, and
`map -> odom -> camera_infra1_optical_frame`.

The simulation launch is:

```text
isaac_ros_visual_slam_d435i_sim.launch.py
```

This milestone is stereo-only. IMU fusion remains disabled until the simulated
optical-to-IMU transform and axis convention are independently verified. It
does not prove real-camera accuracy, motion performance, map persistence, or
relocalization.

## Baseline Square Mapping Result

On 2026-07-28, the formal Level 0 test used a low-speed command profile whose
only inputs were `/scan` and `/visual_slam/status`. Runtime graph inspection
confirmed no ground-truth or odometry subscription. Ground truth was recorded
and read only after the run.

The truth endpoint displacement was `0.6123 m`, while cuVSLAM tracking
odometry reported `8.8072 m`. All `1,342` status samples still reported
`vo_state=1`. This is a failed trajectory-quality result despite correct
estimator-only architecture and process health. Do not increase scene
complexity until stereo calibration, optical-frame extrinsics, image
synchronization, and output frame semantics are isolated. Full evidence is in
`reports/baseline_square_mapping_20260728.md`.

## Drift Isolation And Frame Hypothesis

On 2026-07-29, clean static, straight, and in-place-turn bags isolated the
trajectory failure. Static cuVSLAM was stable. Pure straight motion introduced
scale error, false height, and false rotation before any turn. Rotation then
amplified the failure:

- straight truth `0.2355 m`; cuVSLAM `0.3779 m`, Z `+0.1112 m`,
  rotation `5.59 deg`;
- turn truth `37.70 deg` with zero translation; cuVSLAM `12.47 deg`,
  false translation `1.2963 m`, Z `+0.1578 m`;
- every recorded status sample remained `vo_state=1`.

Stereo timestamps matched exactly and the pair had zero measured vertical
shift. Both CameraInfo projection matrices still had `P[3]=0`; the baseline is
provided by TF.

A controlled physical-camera-frame launch was tested because the installed
cuVSLAM source applies a canonical basis conversion to TF camera extrinsics.
The TF tree and process checks passed, but the first straight-motion quality
gate regressed to cuVSLAM `0.5792 m` for truth `0.2265 m`, with Z
`-0.1567 m` and rotation `11.79 deg`. The turn test was skipped, zero velocity
was published, and the launch was rolled back.

Do not reapply the physical-frame option without new evidence.

The approved CameraInfo option was subsequently implemented. A simulation-only
adapter now keeps the raw right CameraInfo unchanged and publishes
`/cuvslam/input/infra2/camera_info` with `P[3]=-10.416588`, corresponding to
the declared `0.05 m` baseline. cuVSLAM reads its stereo extrinsic from this
corrected CameraInfo, not from a stereo TF.

Static behavior remained stable. Straight-distance overestimation improved
from about 60 percent to about 23 percent, and absolute straight Z error
improved from `0.1112 m` to `0.0626 m`. The rotation gate still failed:
a truth-only `36.73 deg` turn produced only `9.04 deg` estimated rotation,
`1.1374 m` false translation, and `0.1974 m` false Z. The calibration contract
change is retained, but trajectory quality remains unaccepted.

Evidence:

- `reports/cuvslam_drift_isolation_20260729.md`
- `docs/CUVSLAM_SIM_CAMERA_FRAME_FIX_DESIGN.md`
- `/home/tseng/isaac_ros_ws/data/experiments/drift_isolation_20260729_0905`
- `/home/tseng/isaac_ros_ws/data/experiments/frame_fix_20260729_0925`
- `/home/tseng/isaac_ros_ws/data/experiments/camera_info_baseline_20260729_0955`

## Straight Mapping Trial

On 2026-07-29, sparse SLAM visualization was temporarily enabled to test
straight-only mapping before returning to the turning problem. Short and
medium runs both populated `/visual_slam/vis/landmarks_cloud` and maintained
`vo_state=1`, but trajectory and geometry quality failed:

- short truth `0.2475 m`, cuVSLAM `0.3828 m`, 583 final landmarks;
- medium truth `0.4375 m`, cuVSLAM `0.2524 m`, Z `-0.1225 m`,
  rotation `13.80 deg`, 460 final landmarks;
- final landmark X bounds exceeded `10.7 m` in the approximately 6 m arena;
- no tracking-loss, frame-drop, or image-jitter warning was recorded.

The error changed from distance overestimation to underestimation, so it is
not a stable scale factor. The 1 m and 2 m stages were skipped, zero velocity
was published, and sparse visualization was disabled again. See
`reports/cuvslam_straight_mapping_20260729.md`. Bags are stored at:

```text
/home/tseng/isaac_ros_ws/data/experiments/straight_mapping_20260729_1025
```

## Mac Foxglove Viewer

On 2026-07-28, Foxglove desktop 2.57.0 on the Mac was connected to ROS domain
43 through an SSH tunnel. The bridge runs in the existing Humble Isaac ROS
container and binds only to Jetson `127.0.0.1:8765`; Mac
`127.0.0.1:8766` is the local viewing endpoint.

The Mac GUI `Start simulation` button is now the primary entry point. It starts
the dense v3 Gazebo world, waits for stereo-only cuVSLAM tracking with mapping
visualization enabled, opens Foxglove, and enables the low-rate monitor. IMU
fusion remains disabled and wheel odometry is not part of the active bridge.
The standalone Foxglove-only fallback is:

```text
/Users/tsengpochien/Desktop/cuvslam_project/Foxglove Simulation.command
```

The live connection is read-only and limited to simulation, TF, D435i, and
cuVSLAM topics. Foxglove displays ROS data rather than the rendered Gazebo
world. The verified 3D subscriptions include `/tf`, `/tf_static`, and `/scan`.
The simulation launch publishes the missing static transform from
`camera_infra1_optical_frame` to `slam_bot/laser_frame/lidar`, and the red
LaserScan wall contours were visually verified after a cold restart.

The simulation-only `cuvslam_sim_observability` package now adds a known-world
comparison layer:

```text
/simulation/ground_truth_world_markers
  type: visualization_msgs/msg/MarkerArray
  frame: simulation_world_ground_truth
  QoS: reliable, transient local

map -> simulation_world_ground_truth
  owner: world_outline_publisher
```

The Baseline output contains seven SDF-derived wireframes: floor, four walls,
and two feature boxes. The node aligns the simulator world to cuVSLAM `map`
using the first time-synchronized ground-truth pose and camera TF. This is
evaluation-only data. It must not be subscribed to by cuVSLAM, localization,
mapping, costmaps, or Nav2.

The implementation decision, operator procedure, and rollback are recorded in
`deployment/foxglove/README.md`. Test evidence is in
`reports/foxglove_mac_simulation_20260728.md`.

## Sensor Responsibility

The first navigation architecture is:

```text
D435i + IMU  -> cuVSLAM -> visual odometry / localization
LiDAR        -> /scan   -> Nav2 costmap / horizontal obstacle avoidance

dynamic obstacle manager -> slow / stop / local replan / global replan
```

The two sensors do not publish the same interface. They must have separate
topics and frames. The system must have exactly one authoritative publisher
for each localization transform used by the simulation.

Recommended frame tree:

```text
map -> odom -> base_link
                  |-- camera_link
                  `-- lidar_link
```

The LiDAR is the primary horizontal obstacle sensor. The D435i remains the
visual localization sensor and may later provide 3D obstacle information for
objects above or below the LiDAR scan plane.

## Isolation Rules

Run simulation with a dedicated ROS domain, separate from the Isaac ROS
runtime and the old Foxy systems. The recommended simulation domain is `43`.

- Do not source `/home/tseng/slam_ws/install/setup.bash`.
- Do not source `/home/tseng/jetson_slam_ws/install/setup.bash`.
- Do not mount either old workspace into the Isaac ROS container.
- Do not start real-robot motor, serial, or production Nav2 nodes.
- Do not publish a second `/odom -> base_link` transform.
- Do not treat a cuVSLAM landmark map as a 2D occupancy map.
- Keep all replay and generated data under `/home/tseng/isaac_ros_data` or the
  project `datasets/` directory with a manifest.

## First Simulation Milestone

The first milestone is a repeatable sensor-replay test, not a full autonomous
robot test:

1. Start the isolated Isaac ROS shell with `ROS_DOMAIN_ID=43`.
2. Replay or simulate D435i stereo/IMU input.
3. Confirm cuVSLAM publishes the expected pose and tracking status.
4. Replay or simulate LiDAR `/scan` on the same clock.
5. Confirm `map -> odom -> base_link` and `lidar_link` transforms.
6. Add one static obstacle and verify it enters the local costmap.
7. Add one moving obstacle and record whether the robot slows, stops, or
   requests a new path.

The first test should use three scenarios:

| Scenario | Expected behavior |
|---|---|
| Obstacle outside the predicted path | Continue and monitor |
| Obstacle crossing the path with time to react | Slow down or local replan |
| Obstacle blocks the path with no safe route | Stop, then request replanning |

## Dynamic Obstacle Decision Layer

Moving obstacles must not be handled only by blindly inflating a costmap. The
planned decision layer is:

```text
sensor data
  -> obstacle detection
  -> track ID, velocity, and confidence
  -> short-horizon prediction
  -> path-corridor overlap and time-to-collision
  -> behavior policy
  -> velocity limit, stop, or replan
```

Initial behavior states:

```text
CLEAR -> CAUTION -> AVOID
                 \-> EMERGENCY_STOP
```

The policy must include hysteresis so a noisy detection does not rapidly
alternate between replan and continue. A moving object that is travelling
away from the robot or outside the future path corridor should not trigger a
global replan.

## Evidence Required Per Run

Each run needs an experiment ID and a manifest containing:

- source bag or simulator scenario;
- ROS domain and clock policy;
- sensor topics and frame names;
- camera/LiDAR calibration identifiers;
- cuVSLAM configuration;
- map and trajectory identifiers;
- tracking status and dropped-frame observations;
- obstacle decision state transitions;
- result, anomalies, and operator notes.

## Exit Criteria Before Real Hardware

- Three dynamic-obstacle scenarios are repeatable.
- No duplicate localization TF publisher exists.
- Sensor timestamps and frame transforms are verified.
- Tracking loss causes a declared safe behavior.
- The navigation result is not dependent on Gazebo ground truth.
- The replay/simulation procedure can be run from a clean shell without the
  old Foxy workspaces.
