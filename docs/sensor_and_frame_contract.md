# Simulation Frame Contract

Status: NVIDIA-native stereo target validated for pure 90-degree turn  
Change class: C0 Documentation  
Environment: Jetson AGX Orin simulation, ROS domain 43  
Runtime audit date: 2026-07-30  
Scope: Gazebo, `ros_ign_bridge`, cuVSLAM, observability nodes, and Foxglove

## 1. Purpose

This document separates the intended coordinate-frame contract from the
currently observed TF tree. It is the source of truth for frame definitions,
axes, parent relationships, publisher ownership, and purpose.

No frame or TF publisher may be changed solely to make Foxglove look correct.
Any runtime frame change is a C2 interface change and requires a design review,
rollback procedure, and repeatable motion test.

The words used in the audit tables mean:

- **PASS**: verified against the live Jetson ROS graph.
- **FAIL**: live output contradicts the target contract.
- **OPEN**: declared by configuration or message metadata, but not yet proven
  by a live transform or motion test.
- **N/A**: intentionally not part of the authoritative localization tree.

## 2. Axis Conventions

### 2.1 Robot/body convention

All physical robot, LiDAR, and non-optical camera frames use the ROS body
convention:

```text
+X: forward
+Y: left
+Z: up
right-handed
```

This convention applies to `base_link`, `camera_link`,
`camera_infra1_frame`, `camera_infra2_frame`, `camera_imu_frame`, and the
contracted LiDAR frame.

### 2.2 Camera optical convention

Image and CameraInfo messages use the ROS optical convention:

```text
+X: image right
+Y: image down
+Z: optical forward
right-handed
```

This convention applies to `camera_infra1_optical_frame` and
`camera_infra2_optical_frame`.

### 2.3 World, map, and odometry convention

The Gazebo world and the target ROS `map` and `odom` frames are gravity
aligned:

```text
+X/+Y: horizontal plane
+Z: up
right-handed
```

The absolute horizontal heading of `map` is estimator-defined. It must not be
assumed to match the Gazebo world until an explicit evaluation-only alignment
has been calculated.

The project launch now configures cuVSLAM to use `base_link` as its output
base and `camera_infra1_frame` as the physical left-camera input frame.
The live `map` and `odom` axis semantics remain **OPEN** until deployment and
a controlled motion test prove that they are gravity-aligned ROS navigation
frames.

## 3. Target Authoritative Tree

The long-term frame contract is:

```text
map
`-- odom
    `-- base_link
        |-- camera_link
        |   |-- camera_infra1_frame
        |   |   `-- camera_infra1_optical_frame
        |   |-- camera_infra2_frame
        |   |   `-- camera_infra2_optical_frame
        |   `-- camera_imu_frame
        `-- lidar_link

map
`-- simulation_world_ground_truth    evaluation only
```

Exactly one component owns each edge. In particular:

- cuVSLAM owns `map -> odom`.
- cuVSLAM, or one explicitly approved pose adapter, owns
  `odom -> base_link`.
- Robot description/static extrinsics own all
  `base_link -> sensor frame` edges.
- `world_outline_publisher` alone owns
  `map -> simulation_world_ground_truth`.
- Foxglove owns no TF edge.

## 4. Frame Definitions

| Frame | Origin | Axes | Contract parent | Contract publisher | Purpose |
|---|---|---|---|---|---|
| Gazebo world | SDF world origin | gravity aligned, +Z up | none | Gazebo | Simulation physics and model ground truth; not an estimator input frame |
| `slam_bot/odom` | Diff-drive odometry origin at simulation start | body/navigation convention | none in authoritative TF | not bridged | Prohibited wheel-odometry frame; not an estimator or truth source |
| `slam_bot/base_link` | Robot chassis reference point; SDF model origin | +X forward, +Y left, +Z up | none in authoritative ROS TF | not bridged | Prohibited wheel-odometry child frame; native Gazebo model pose is used offline instead |
| `map` | cuVSLAM local/global map origin at estimator initialization | expected gravity aligned; live semantics OPEN | none | cuVSLAM `visual_slam_node` | Global visual mapping and localization reference |
| `odom` | Continuous cuVSLAM odometry origin | expected gravity aligned; live semantics OPEN | `map` | cuVSLAM `visual_slam_node` | Locally continuous estimator reference |
| `base_link` | Robot chassis reference point at the SDF model origin | +X forward, +Y left, +Z up | `odom` | cuVSLAM or approved pose adapter | Navigation pose reference and root of physical sensor extrinsics |
| `camera_link` | D435i body center, SDF pose `(0.19, 0, 0.20)` from `base_link` | +X forward, +Y left, +Z up | `base_link` | Robot description/static TF owner | Physical D435i body reference |
| `camera_infra1_frame` | Left imager center; `(0, +0.025, 0)` from `camera_link` | +X forward, +Y left, +Z up | `camera_link` | Robot description/static TF owner | Physical left infrared camera frame |
| `camera_infra2_frame` | Right imager center; `(0, -0.025, 0)` from `camera_link` | +X forward, +Y left, +Z up | `camera_link` | Robot description/static TF owner | Physical right infrared camera frame |
| `camera_infra1_optical_frame` | Same physical origin as `camera_infra1_frame` | +X right, +Y down, +Z forward | `camera_infra1_frame` | Robot description/static TF owner | `infra1` image/CameraInfo frame and left stereo reference |
| `camera_infra2_optical_frame` | Same physical origin as `camera_infra2_frame` | +X right, +Y down, +Z forward | `camera_infra2_frame` | Robot description/static TF owner | `infra2` image/CameraInfo frame and right stereo reference |
| `camera_imu_frame` | IMU origin; currently colocated with `camera_link` in SDF | +X forward, +Y left, +Z up | `camera_link` | Robot description/static TF owner | Simulated IMU measurement frame; fusion is disabled by default |
| `slam_bot/camera_imu_frame/d435i_imu` | Current Gazebo scoped IMU sensor origin | expected body convention | currently disconnected | none observed | Temporary live `/d435i/imu` message frame; must be normalized before fusion |
| `lidar_link` | LiDAR scan center; SDF pose `(0, 0, 0.13)` from `base_link` | +X forward, +Y left, +Z up | `base_link` | Robot description/static TF owner | Contracted horizontal LaserScan frame for future costmaps |
| `slam_bot/laser_frame/lidar` | Current Gazebo scoped LiDAR sensor origin | expected LaserScan body convention | currently `camera_infra1_optical_frame` | `simulated_lidar_extrinsic` | Temporary live `/scan` frame; must eventually be normalized to `lidar_link` |
| `simulation_world_ground_truth` | Gazebo world origin transformed into the initial cuVSLAM map | Gazebo world axes | `map` | `world_outline_publisher` | Evaluation-only SDF outline in Foxglove |

## 5. Numeric Extrinsic Contract

Values below come from
`simulation/cuvslam/deployment/slam_gazebo/worlds/cuvslam_mapping_simple.sdf`.

For the Baseline world, the same sensor values were independently verified
against `indoor_gz_sim.sdf` using three absolute stationary targets and three
fresh validator processes on 2026-07-30. All 72 target/frame comparisons
passed with zero residual at the reported precision. These static numeric
values are **LOCKED** for the current SDF versions. See
`simulation/cuvslam/evidence/reports/static_tf_target_validation_20260730.md`.

The project launch contains this locked static geometry. Jetson runtime
activation and the three-run motion acceptance test are still required before
the configuration is accuracy-qualified.

| Transform | Translation (m) | Rotation | Meaning |
|---|---:|---|---|
| `base_link -> camera_link` | `(0.19, 0, 0.20)` | identity | Camera body is forward and above chassis origin |
| `camera_link -> camera_infra1_frame` | `(0, +0.025, 0)` | identity | Left imager |
| `camera_link -> camera_infra2_frame` | `(0, -0.025, 0)` | identity | Right imager |
| `camera_infra1_frame -> camera_infra2_frame` | `(0, -0.05, 0)` | identity | Physical stereo baseline |
| left optical -> right optical | `(+0.05, 0, 0)` in optical coordinates | identity after rectification | CameraInfo stereo baseline |
| `base_link -> camera_imu_frame` | `(0.19, 0, 0.20)` | identity | Current simulated IMU placement |
| `base_link -> lidar_link` | `(0, 0, 0.13)` | identity | Horizontal LiDAR scan plane |

For the right rectified CameraInfo:

```text
baseline B = 0.05 m
fx = 208.331766...
P[3] = -fx * B = -10.416588...
R = identity
```

The physical-to-optical rotation must be represented exactly once in TF. It
must not be duplicated in CameraInfo, cuVSLAM input parameters, or a display
adapter.

## 5.1 Wheel-Odometry Isolation Contract

Gazebo DiffDrive may accept `/cmd_vel` to actuate the simulated chassis, but
its odometry message and TF are not authoritative and must not enter the
bridge, recorder, cuVSLAM, controller, Nav2, or evaluator.

`/ground_truth/odom` is a prohibited legacy name because the underlying data
is wheel integration rather than physical ground truth. The active runner
fails if this topic is present. Physical simulation truth is read directly
from Gazebo native dynamic pose and is used only for test control or offline
evaluation.

## 6. Previous Live Jetson Tree

Captured before the NVIDIA-native architecture selection on 2026-07-30 using
`tf2_tools view_frames` in
`isaac_ros_dev-aarch64-container`, ROS domain 43:

```text
map
|-- odom
|   `-- camera_infra1_optical_frame
|       `-- slam_bot/laser_frame/lidar
`-- simulation_world_ground_truth
```

Previous live publishers:

| Edge | Actual publisher | Type/rate | Audit |
|---|---|---|---|
| `map -> odom` | `/visual_slam_node` | dynamic, about 30.55 Hz | PASS for ownership; axes OPEN |
| `odom -> camera_infra1_optical_frame` | `/visual_slam_node` | dynamic, about 30.55 Hz | FAIL against target `odom -> base_link` |
| `camera_infra1_optical_frame -> slam_bot/laser_frame/lidar` | `/simulated_lidar_extrinsic` | static | PASS for current visualization; FAIL against normalized target tree |
| `map -> simulation_world_ground_truth` | `/world_outline_publisher` | static | PASS, evaluation only |

Observed static LiDAR transform:

```text
translation: (0.025, 0.070, -0.190) m
quaternion:  (0.5, -0.5, 0.5, 0.5)
RPY:         (0, -90 deg, 0)
```

Publisher counts:

- `/tf`: one publisher, `/visual_slam_node`.
- `/tf_static`: three publishers: `/visual_slam_node`,
  `/simulated_lidar_extrinsic`, and `/world_outline_publisher`.
- No duplicate edge was observed in the captured tree.

At the stationary sample, both `map -> odom` and
`odom -> camera_infra1_optical_frame` were identity. This is not a motion
accuracy result.

### 6.1 Motion audit

The 2026-07-30 frame-aware replay produced:

- straight ground truth: `base_link +X = 0.2270 m`;
- expected left-optical motion: optical `+Z = 0.2270 m`;
- cuVSLAM output labelled as left optical:
  `(+0.2711, +0.0231, -0.0626) m`;
- in-place 36.73 degree turn: expected camera arc `0.1207 m`, cuVSLAM
  translation norm `1.1374 m`, and cuVSLAM rotation `9.04 deg`.

The dominant straight-motion axis is inconsistent with the declared optical
child frame. This confirms that the live output frame name and numeric axis
semantics do not satisfy this contract.

The valid camera lever arm explains part of the turn translation, but its
expected 0.1207 m arc cannot explain the observed 1.1374 m magnitude. See
`simulation/cuvslam/evidence/reports/frame_contract_motion_audit_20260730.md`.

## 7. Historical Gazebo, Bridge, cuVSLAM, and Foxglove Cross-check

This table records the pre-change runtime audit. Its failures are the reason
for the NVIDIA-native launch migration; it is not a description of the new
launch configuration.

| Item | Gazebo/SDF declaration | Bridge/message output | cuVSLAM/runtime TF | Foxglove behavior | Audit |
|---|---|---|---|---|---|
| Robot body | `base_link` exists at model origin | `/ground_truth/odom`: parent `slam_bot/odom`, child `slam_bot/base_link` | no `base_link` frame | cannot display body through authoritative TF | FAIL |
| Camera body | `camera_link` at `(0.19,0,0.20)` | no TF edge observed | absent | camera body cannot be placed from TF | FAIL |
| Left physical camera | `camera_infra1_frame`, +0.025 m Y | no TF edge observed | absent | not selectable as connected frame | FAIL |
| Right physical camera | `camera_infra2_frame`, -0.025 m Y | no TF edge observed | absent | not selectable as connected frame | FAIL |
| Left optical camera | sensor declares `camera_infra1_optical_frame` | left image/CameraInfo use this `frame_id` | used directly as cuVSLAM base and output child | visible and connected | OPEN: message identity exists, physical parent edge missing |
| Right optical camera | sensor declares `camera_infra2_optical_frame` | adapted CameraInfo preserves this `frame_id` | no right optical TF in tree; baseline comes from CameraInfo | topic is visible but frame is disconnected | FAIL |
| Stereo baseline | imager origins are 0.05 m apart | corrected right `P[3]` encodes 0.05 m | explicit stereo TF input disabled | images display independently | PASS for CameraInfo; TF geometry OPEN |
| IMU | `camera_imu_frame` exists | `/d435i/imu.frame_id = slam_bot/camera_imu_frame/d435i_imu` | fusion disabled by default; scoped alias is connected by static TF | opt-in fusion trial is available but rejected for normal use | PASS for connectivity; FAIL for estimator-behavior acceptance |
| LiDAR | `laser_frame` at `(0,0,0.13)` | `/scan.frame_id = slam_bot/laser_frame/lidar` | temporary static edge from left optical frame | red scan contour is connected | PASS for display, FAIL for target naming/parent |
| Ground truth | DiffDrive publishes model odometry | message frames are scoped Gazebo names | intentionally not inserted into localization TF | available for evaluation panels | PASS |
| World outline | SDF geometry is parsed | MarkerArray uses `simulation_world_ground_truth` | aligned once under `map` | cyan world outline displays | PASS, evaluation only |
| `map -> odom` | not owned by Gazebo | none | cuVSLAM is sole owner | consumed read-only | PASS |
| `odom -> base` | Gazebo truth exists only as a message | scoped truth frame names | cuVSLAM publishes to left optical frame, not `base_link` | optical camera appears as tracked body | FAIL |

Foxglove is a read-only consumer in this architecture. It can reveal a
disconnected or misaligned tree, but it does not create the localization TF
error.

## 8. Historical Contract Gaps And Migration Status

### GAP-01: Navigation body frame was absent

The target is `odom -> base_link`. The audited tree was
`odom -> camera_infra1_optical_frame` because the launch sets:

```text
base_frame = camera_infra1_optical_frame
input_base_frame = camera_infra1_optical_frame
```

The project launch now sets `base_frame` and `input_base_frame` to `base_link`.
Runtime verification remains pending.

### GAP-02: Gazebo links were not ROS TF frames

The project launch now publishes the locked static tree containing
`base_link`, `camera_link`, both physical and optical infrared frames, the IMU
frames, and the LiDAR frames. Message `frame_id` strings still do not create
TF edges; the launch publishers explicitly own them.

### GAP-03: Right image calibration and TF were partially contracted

The corrected right CameraInfo remains the sole cuVSLAM stereo-baseline
authority. The new static tree connects `camera_infra2_optical_frame` for
auditing and visualization, while `input_right_camera_frame` remains empty.

### GAP-04: LiDAR uses a scoped simulator frame

The `/scan` header still uses `slam_bot/laser_frame/lidar`. The new static tree
connects that simulation alias under `lidar_link`; `lidar_link` remains the
normalized navigation frame.

### GAP-05: IMU uses a scoped simulator frame

The `/d435i/imu` header uses
`slam_bot/camera_imu_frame/d435i_imu`, while the SDF link and target contract
use `camera_imu_frame`. The new static tree connects the scoped alias under
`camera_imu_frame`. IMU fusion remains disabled by default. The 2026-07-30
Stage 4 opt-in trial retained tracking but estimated `1.6242 m` for `0.4895 m`
of native 6DoF straight motion and added `22.99 deg` of false rotation. It
therefore remains a diagnostic-only launch option.

### GAP-06: Ground-truth and estimator reference points differed

Ground truth measures `slam_bot/base_link`. The new cuVSLAM configuration
estimates `base_link`, allowing like-for-like evaluation after the scoped
ground-truth message is normalized by the evaluator. The prior camera-frame
bags remain historical evidence and must still account for the lever arm.

## 9. Validation Procedure

All checks use ROS domain 43. Save command output with the experiment bundle.

### 9.1 Graph and ownership

```bash
ros2 node list
ros2 topic info /tf -v
ros2 topic info /tf_static -v
ros2 run tf2_tools view_frames --wait-time 5 --output frame_contract
```

Pass criteria:

- one publisher per TF edge;
- all target frames are connected;
- no Gazebo ground-truth edge enters the authoritative localization tree;
- Foxglove is not a publisher.

### 9.2 Message frame IDs

```bash
ros2 topic echo --once /d435i/infra1/camera_info
ros2 topic echo --once /cuvslam/input/infra2/camera_info
ros2 topic echo --once /d435i/imu
ros2 topic echo --once /scan
ros2 topic echo --once /ground_truth/odom
ros2 topic echo --once /visual_slam/tracking/odometry
```

Pass criteria:

- each message `frame_id` exists in TF;
- left and right image and CameraInfo pairs use matching optical frames;
- odometry parent and child frames match their declared data source;
- simulation ground truth remains distinguishable by scoped names.

### 9.3 Static extrinsics

Verify every edge numerically with `tf2_echo`:

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo camera_link camera_infra1_frame
ros2 run tf2_ros tf2_echo camera_link camera_infra2_frame
ros2 run tf2_ros tf2_echo camera_infra1_frame camera_infra1_optical_frame
ros2 run tf2_ros tf2_echo camera_infra2_frame camera_infra2_optical_frame
ros2 run tf2_ros tf2_echo base_link lidar_link
```

Pass criteria:

- translations match Section 5 within numerical precision;
- both physical-to-optical rotations are identical;
- left-to-right optical translation is +0.05 m on optical X;
- all static transforms have one owner and timestamp zero.

### 9.4 Controlled motion

Record three clean-start runs:

1. Stationary for 10 seconds.
2. Straight translation for at least 1 m.
3. In-place chassis yaw of approximately 30 degrees.

For every timestamp compare:

- Gazebo `slam_bot/base_link`;
- Gazebo left optical pose derived from the static contract;
- raw cuVSLAM left optical pose;
- cuVSLAM pose transformed back to `base_link`.

Pass criteria:

- stationary pose remains stable;
- straight motion is predominantly horizontal and forward;
- pure chassis yaw contains only the predicted camera lever-arm arc;
- after transforming both sources to `base_link`, residual translation during
  in-place yaw is near zero;
- no axis swap, sign reversal, or discontinuity appears.

## 10. Change Gate

The two implementation options were compared in
`CUVSLAM_TF_CONTRACT_FIX_DESIGN.md`. The NVIDIA-native `base_frame` mechanism
was selected; an external pose adapter was rejected for the normal runtime.

Do not modify cuVSLAM algorithm parameters until the new launch passes TF
ownership, static-edge, late-subscriber, and visualization checks. Trajectory
accuracy remains a separate gate: three independent trials must each stay
within the declared 15 percent threshold and absolute drift limits.
