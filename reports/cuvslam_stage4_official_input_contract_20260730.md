# cuVSLAM Stage 4 Official D435i Input Contract

Date: 2026-07-30  
Environment: Jetson AGX Orin simulation, ROS domain 43  
cuVSLAM: Isaac ROS Visual SLAM 2.1.0, cuVSLAM 11.4  
Result: 6DoF truth correction accepted; simulated IMU fusion rejected  
Final runtime state: Simulation stopped

## Scope

This stage compared the installed NVIDIA D435i and Isaac Sim launch contracts
with the current simulated D435i path. Camera geometry, TF, world, image rate,
and cuVSLAM noise parameters were not retuned.

The installed D435i example uses rectified infrared stereo plus 200 Hz IMU
fusion. The installed Isaac Sim example confirms that rectified stereo without
IMU is supported. The current 424 x 240, 30 Hz, `mono8` stereo path is therefore
a supported architecture, although it is not identical to the complete D435i
example.

## Contract Audit

| Contract item | Current simulation | Installed NVIDIA example | Decision |
|---|---|---|---|
| Rectified stereo | true | true | match |
| Stereo baseline | right CameraInfo `P[3]`, 0.05 m | CameraInfo | match |
| Left/right stamps | exact sim-time pairs | synchronized sensor pair | match |
| Image format | `mono8` infrared | infrared | compatible |
| Output frame | `base_link` through official parameters | configurable base frame | match |
| IMU rate | 200 Hz sim-time | 200 Hz | match in sim-time |
| IMU fusion | false by default | true for D435i; false in Isaac Sim launch | A/B required |
| D435i image profile | 424 x 240 x 30 | installed 2.1 launch uses 640 x 360 x 90 | intentional difference |

The official cuVSLAM noise defaults already matched the current node. The IMU
message frame is connected to the left camera through the locked static TF
tree. No runtime TF was modified.

## Ground-Truth Defect

Gazebo native `dynamic_pose/info` showed the physical `slam_bot` model resting
at approximately `13.8 deg` pitch. `/ground_truth/odom` reported identity
orientation because it is the diff-drive plugin's planar odometry, not a 6DoF
physical pose.

The Stage 4 evaluator now accepts the Gazebo Transport text log directly,
preserving entity name and simulation timestamp. A direct `Pose_V` ROS bridge
was rejected because the installed Foxy bridge drops both fields.

Using native 6DoF truth and the correct cuVSLAM `base_link` output reference:

| Phase | Native truth | Stereo-only estimate | Result |
|---|---:|---:|---|
| Static | 0 motion | 0 motion | pass |
| Straight distance | 0.4433 m | 0.3542 m | 20.1% low |
| Straight rotation | 0.01 deg | 2.61 deg | fail |
| Turn rotation | 25.72 deg | 26.12 deg | 1.6% magnitude error |
| Turn rotation residual | - | 0.41 deg | pass |
| Turn translation | 0.0000 m | 0.3562 m | fail |

The truth correction reduced the turn rotation residual from `6.15 deg` with
planar odometry to `0.41 deg`. This proves that most previously reported turn
rotation error came from the evaluation reference. It does not explain the
remaining false translation.

## Official-Style IMU A/B

An opt-in launch argument enabled IMU fusion with the exact message frame,
200 Hz calibration frequency, and NVIDIA default noise parameters. Tracking
remained `vo_state=1`, but motion regressed:

| Phase | Native truth | IMU estimate | Result |
|---|---:|---:|---|
| Static | 0 motion | 0 motion | pass |
| Straight distance | 0.4895 m | 1.6242 m | 231.8% high |
| Straight rotation | 0.04 deg | 22.99 deg | fail |
| Turn rotation | 24.52 deg | 24.46 deg | 0.2% magnitude error |
| Turn rotation residual | - | 0.12 deg | pass |
| Turn translation | 0.0000 m | 0.2947 m | fail |

IMU fusion is rejected for normal use. The launch keeps it available only as
an explicit A/B diagnostic, with `false` as the default. A clean rollback
startup confirmed `enable_imu_fusion=False` and `vo_state=1`.

## Stage Decision

Accepted:

- evaluate camera motion against Gazebo-native 6DoF truth;
- infer `base_link` output reference from the odometry child frame;
- retain the opt-in IMU diagnostic with a stereo-only default.

Rejected:

- planar `/ground_truth/odom` as 6DoF accuracy truth;
- enabling simulated IMU fusion as a cuVSLAM correction;
- changing the locked runtime TF tree.

The remaining Stage 4 defect is specifically visual translation, not turn
angle: straight motion has a 20.1% scale deficit and 2.61 deg false rotation,
while an in-place turn has 0.3562 m false translation despite a correct angle.
The next experiment should inspect cuVSLAM's camera-axis translation output
against image-derived optical flow and native camera pose, without changing TF.

## Evidence

- `docs/CUVSLAM_STAGE4_OFFICIAL_INPUT_CONTRACT_DESIGN.md`
- `tools/gz_pose_log.py`
- `tools/test_gz_pose_log.py`
- `tools/evaluate_frame_contract_bag.py`
- `reports/stage4_static_dynamic_pose_sample.txt`
- `reports/stage4_official_contract_20260730/`

Local tests:

```text
Ran 9 tests
OK
```

## Final Shutdown Evidence

```text
cuvslam-stage4-truth.service: inactive
cuvslam-stage4-bridge.service: inactive
cuvslam-stage4-gazebo.service: inactive
cuVSLAM launch/component/helper processes: none
/clock: no message during a 3-second sample
```
