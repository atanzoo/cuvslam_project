# Real D435i R2 stereo-only trial — 2026-08-18

## Configuration

- Device: Intel RealSense D435i, serial `261222077990`, firmware `5.15.1.55`
- Jetson: AGX Orin, JetPack 5.1.3 / CUDA 11.4
- cuVSLAM: 11.4 through Isaac ROS Visual SLAM 2.1.0
- Image streams: left/right `848x480`, mono8, 30 Hz
- IMU stream: accel + gyro, nominal 200 Hz
- `enable_mapping=true`
- `enable_imu_fusion=false`
- `initial_reset=false` for R2 startup after the USB recovery
- Output frames: `odom -> camera_link`

## Trial

- Run directory on Jetson: `/home/tseng/isaac_ros_data/experiments/real_d435i_r2/r2_stereo_only_straight_20260818/`
- Route label: `manual_short_straight`
- Duration: 30.473 s in the recorded bag
- Recorded topics: stereo images/CameraInfo, IMU, `/tf`, `/tf_static`, cuVSLAM status, odometry, VO pose, SLAM path, and landmarks cloud

## Bag-level result

| Metric | Result |
|---|---:|
| Left image messages | 910 |
| Right image messages | 910 |
| IMU messages | 6,049 (~200 Hz) |
| Odometry messages | 807 |
| Status messages | 807 |
| `vo_state=1` | 807/807 (100%) |
| End displacement | 0.7028 m |
| Path length | 1.1640 m |
| Start-to-end rotation | 14.37° |

## Assessment

The real stereo-only cuVSLAM pipeline, mapping-enabled launch, rosbag
recording, and odometry/status outputs all passed this first trial. The route
was not a pure straight-line baseline because the camera accumulated about
14° of rotation and the path length exceeded the end displacement. Repeat the
same route with a more constrained heading before using it as the final
straight-line accuracy measurement.

The local copies of the trial JSON and bag metadata are under
`logs/real_d435i_r2/r2_stereo_only_straight_20260818/`; the large rosbag
remains on the Jetson experiment directory.
