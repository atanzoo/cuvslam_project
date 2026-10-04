# cuVSLAM D435i IMU Corner Audit

Date: 2026-07-31  
Platform: Jetson AGX Orin, Isaac ROS Visual SLAM 2.1.0 / cuVSLAM 11.4  
Result: D435i IMU contract passed; wheel-odometry turn gate fixed; simulated
IMU fusion did not improve cuVSLAM

## NVIDIA Reference

The NVIDIA cuVSLAM paper does not prescribe a commanded
`1 m -> 90 deg -> 0.5 m` route. The comparable contract is the live
stereo-inertial sensor path:

- RealSense D435/D455;
- 640 x 480 stereo at 30 FPS;
- IMU at 200 Hz;
- approximately 6--7 IMU measurements between stereo pairs.

The installed Isaac ROS 2.1 RealSense launch also enables IMU fusion and uses
200 Hz gyro and accelerometer streams. Its image profile is
`640 x 360 x 90`, so it is an implementation example rather than the paper's
exact benchmark profile.

References:

- <https://arxiv.org/html/2506.04359>
- <https://nvidia-isaac-ros.github.io/v/release-2.1/concepts/visual_slam/cuvslam/index.html>
- <https://nvidia-isaac-ros.github.io/v/release-2.1/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html>

## Input Contract Audit

The current level-chassis simulation produced:

| Item | Measured |
|---|---:|
| Left image rate | 28.57 Hz |
| IMU rate | 200.00 Hz |
| IMU samples per image interval | median 7, P05 6 |
| Image/IMU nearest skew | median 0 ms, P95 0 ms |
| IMU frame | `slam_bot/camera_imu_frame/d435i_imu` |
| Acceleration norm | 9.8000 m/s^2 |

The camera is declared as 30 Hz, but the 5 ms Gazebo physics step quantizes
the measured interval to 35 ms, or 28.57 Hz. The 200 Hz IMU and 6--7 samples
per image match the paper's inertial cadence.

The launch now explicitly carries the NVIDIA D435i IMU noise parameters and
200 Hz calibration frequency. The bag recorder now includes `/d435i/imu`.

## Ground-Truth Correction

The route controller previously stopped the turn when
`/ground_truth/odom` reached 90 degrees. That topic is the Gazebo DiffDrive
plugin's planar wheel odometry, not the model's physical world pose.

In the same turn interval:

| Source | Angle / rate |
|---|---:|
| Wheel odometry turn | about 91 deg |
| Gazebo native model turn | 79.28 deg |
| IMU integrated turn | 79.29 deg |
| Native yaw rate | 12.67 deg/s |
| IMU median yaw rate | 12.67 deg/s |

The native-pose and IMU angle differed by only 0.01 degree. Wheel slip caused
the wheel odometry to overstate the physical turn by about 11 degrees.

The runner now records Gazebo Transport `dynamic_pose/info`. The turn
controller now gates completion using timestamped D435i gyro integration,
with QoS depth 1 and callback draining to avoid stale IMU queues.

The corrected short validation produced:

| Source | Turn |
|---|---:|
| Controller IMU accumulator | 90.79 deg |
| Gazebo native model pose | 89.74 deg |
| Bag IMU integration over native turn | 89.76 deg |
| Wheel odometry | 103.92 deg |

The corrected turn program therefore reaches a physical 90-degree turn within
0.26 degree by native truth. `/ground_truth/odom` must not be used as the turn
acceptance reference.

## Stereo-Only Versus VIO A/B

Both cold-start trials used the same world and nominal route. Native physical
turns were approximately 80 degrees because these A/B runs preceded the
IMU-gated route-controller correction.

| Metric | Stereo only | IMU fusion |
|---|---:|---:|
| Tracking state | all state 1 | all state 1 |
| Native turn | 79.28 deg | 79.37 deg |
| Estimated turn | 68.22 deg | 67.13 deg |
| Turn rotation residual | 11.28 deg | 12.72 deg |
| Turn false translation | 0.2738 m | 0.3251 m |
| Second-leg translation error | 0.4359 m | 0.4426 m |
| Total translation residual | 0.3386 m | 0.2278 m |

The lower VIO total translation residual is cancellation across phases; VIO
was worse during the turn and second leg, so it is not accepted as a fix.

Final landmark geometry:

| Metric | Stereo only | IMU fusion |
|---|---:|---:|
| Landmarks | 3,065 | 3,103 |
| Median surface distance | 0.0589 m | 0.0762 m |
| P90 surface distance | 0.8221 m | 0.7864 m |
| Within 0.25 m | 65.8% | 66.6% |
| Maximum surface distance | 18.33 m | 24.73 m |

VIO slightly improved P90 and the 0.25 m pass fraction, but worsened the
median, maximum outlier, turn angle, and false turn translation. This is not
a meaningful geometry-quality improvement.

## Decision

- Keep `enable_imu_fusion=false` as the normal simulation default.
- Keep VIO available as an explicit A/B diagnostic.
- Use the corrected IMU-gated controller for future 90-degree routes.
- Use Gazebo native model pose as independent trajectory truth.
- Treat the remaining defect as visual/VIO translation and rotation
  estimation under turning, not D435i IMU calibration.
- Resolve the measured 28.57 Hz camera cadence before claiming an exact
  30 FPS paper-profile reproduction.

## Evidence Bundles

- Stereo control:
  `/home/tseng/isaac_ros_ws/data/corner_native_truth_stereo_20260731_63912`
- VIO:
  `/home/tseng/isaac_ros_ws/data/corner_native_truth_vio_20260731_65512`
- Corrected 90-degree controller:
  `/home/tseng/isaac_ros_ws/data/corner_imu_gated_validation_drain_20260731_72016`

