# D435i Simulation Sensor Source Report

Experiment ID: `D435I-SIM-SENSOR-20260727-01`

Date: 2026-07-27 (Asia/Taipei)

## Scope

Add a simulation-only D435i-equivalent stereo and IMU source to the existing
Jetson `slam_gazebo` package. This change does not start or modify cuVSLAM,
LiDAR ICP, Nav2, motor, serial, or real-robot nodes.

## Environment

- Jetson Linux R35.5.0 / Ubuntu 20.04 / aarch64
- Host ROS 2 Foxy with `rmw_fastrtps_cpp`
- Ignition Gazebo Fortress 6.16.0
- `ros_ign_bridge` 0.221.2
- Simulation workspace: `/home/tseng/jetson_slam_ws`
- ROS domain: `43`
- Clock policy: Gazebo `/clock`
- Power mode observed during tests: 30 W

## Changed Files

- `/home/tseng/jetson_slam_ws/src/slam_gazebo/worlds/indoor_gz_sim.sdf`
- `/home/tseng/jetson_slam_ws/src/slam_gazebo/launch/gz_sim.launch.py`
- `/home/tseng/jetson_slam_ws/src/slam_gazebo/launch/gz_bridge_portable.launch.py`

Installed source and package-share copies had matching SHA-256 hashes:

- world: `ec72783670920977ec5dc4e0cd1363f7aeb879e05401bd771c2bf3951b90a004`
- Gazebo launch: `1f6a315d8abfd8d52960e81633cdb128592f0dbc2c80b0e458e72143b4cf498f`
- bridge launch: `2c84d5c6563b60bccb9a9396e34615b51c7bd0da777a1c636bb9b3bf5b20c57d`

## Sensor Contract

| Topic | Type | Simulated rate | Frame |
|---|---|---:|---|
| `/d435i/infra1/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz | `camera_infra1_optical_frame` |
| `/d435i/infra1/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | `camera_infra1_optical_frame` |
| `/d435i/infra2/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz | `camera_infra2_optical_frame` |
| `/d435i/infra2/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | `camera_infra2_optical_frame` |
| `/d435i/imu` | `sensor_msgs/msg/Imu` | 200 Hz | `slam_bot/camera_imu_frame/d435i_imu` |

Stereo configuration:

- `mono8`, 424 x 240
- horizontal field of view: 1.58825 rad
- stereo baseline: 0.050 m
- zero simulated lens distortion
- focal length from generated CameraInfo: approximately 208.33 px
- physics step: 0.005 s

## Verification

- Local Python launch syntax: passed.
- Local XML well-formed check: passed.
- Jetson `ign sdf -k`: `Valid`.
- Fortress headless OGRE2 startup: passed.
- `colcon build --packages-select slam_gazebo --symlink-install`: passed.
- ROS topic type and publisher checks: passed for both images, both
  CameraInfo topics, IMU, and `/clock`.
- Image content check: `mono8`, 424 x 240, step 424.
- CameraInfo check: left and right intrinsics match; frame IDs are distinct.
- IMU content check: orientation, angular velocity, linear acceleration, and
  simulation timestamps are populated.
- Dual image subscriber wall-clock observation: approximately 25-27 Hz.
- Gazebo process shutdown: clean.
- Fortress `parameter_bridge` shutdown: required SIGTERM after the launch
  system's five-second SIGINT grace period; no process was intentionally left
  running.

## Known Limitations

- Wall-clock stereo delivery is below the 30 Hz target under dual ROS
  subscribers in 30 W mode, although sensor timestamps advance at 30 Hz.
- No static TF publisher exists yet for the camera and IMU frames.
- Stereo timestamp delta has not yet been recorded by a dedicated synchronizer.
- cuVSLAM tracking, odometry, status, trajectory, and resource metrics have not
  been tested in this change unit.
- Gazebo ground truth is evaluation-only and must not be fed into cuVSLAM.

## Rollback

Backups were created before deployment:

- `indoor_gz_sim.sdf.bak.20260727_142955`
- `gz_sim.launch.py.bak.20260727_142955`
- `gz_bridge_portable.launch.py.bak.20260727_142955`

Restore only these exact backups, then rebuild `slam_gazebo`, if this sensor
source must be rolled back.

## Next Change Unit

In the independent Isaac ROS workspace, add one simulation-specific cuVSLAM
launch contract that:

- remaps the five D435i topics to cuVSLAM inputs;
- publishes the required static camera and IMU transforms;
- uses `ROS_DOMAIN_ID=43` and `use_sim_time=true`;
- enables IMU fusion;
- verifies stereo timestamp delta before declaring tracking success;
- records `visual_slam/status`, odometry, dropped frames, GPU load, and
  temperature.
