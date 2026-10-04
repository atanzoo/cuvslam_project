# cuVSLAM D435i Simulation Integration

Date: 2026-07-27

## Scope

Change class: C2 simulation integration.

This change connects the declared simulated D435i stereo topics to the existing
Isaac ROS 2.1 `isaac_ros_visual_slam` package in the isolated Humble container.
It does not modify production navigation, the real camera path, or the existing
D435i topic contract.

Acceptance signals:

- cuVSLAM SDK 11.4 creates a tracker.
- `/visual_slam/status` publishes `vo_state: 1`.
- `/visual_slam/tracking/odometry` publishes a stamped pose.
- `map -> odom -> camera_infra1_optical_frame` is queryable.
- GUI Start, Status, Stop, and dependency shutdown are repeatable.

## Changed Files

- `deployment/isaac_ros/isaac_ros_visual_slam_d435i_sim.launch.py`
  - Launches only the cuVSLAM component and a declared stereo static transform.
  - Remaps the existing `/d435i/infra1`, `/d435i/infra2`, and `/d435i/imu` topics.
  - Uses ROS domain 43 and simulated time.
  - Publishes `map -> odom -> camera_infra1_optical_frame`.
  - Runs stereo tracking with IMU fusion disabled.
- `tools/jetson_connection_web.py`
  - Adds Start cuVSLAM, Stop cuVSLAM, and cuVSLAM status.
  - Reports success only after a real `vo_state: 1` sample.
  - Stops cuVSLAM before stopping the D435i bridge and Gazebo.
  - Manages the launch through detached `docker exec` and verifies the container PID.

Remote deployed file:

```text
/home/tseng/isaac_ros_ws/src/isaac_ros_visual_slam/
  isaac_ros_visual_slam/launch/isaac_ros_visual_slam_d435i_sim.launch.py
```

## Stereo Extrinsic

The Gazebo right-camera `CameraInfo.P[3]` is zero, so cuVSLAM cannot infer a
baseline from CameraInfo. The SDF declares the left and right sensors at
`y=+0.025 m` and `y=-0.025 m`. The launch therefore publishes:

```text
camera_infra1_optical_frame -> camera_infra2_optical_frame
translation: +0.05, 0, 0 m
rotation: identity
```

cuVSLAM reads the stereo extrinsic from this TF instead of CameraInfo.

## Verification

Syntax and focused build:

```text
Python launch syntax: PASS
Python GUI server syntax: PASS
colcon build --symlink-install --packages-select isaac_ros_visual_slam
Result: 1 package finished
```

Live tracker:

```text
cuVSLAM version: 11.4
left_pose_right reading from TF Tree
Enable IMU Fusion: false
CUVSLAM_CreateTracker(): success
vo_state: 1
```

Odometry sample:

```text
frame_id: odom
child_frame_id: camera_infra1_optical_frame
position: [-0.0072, 0.0132, -0.0077] m
```

TF:

```text
map -> odom: query succeeded
odom -> camera_infra1_optical_frame: query succeeded
```

GUI lifecycle:

```text
Start cuVSLAM: vo_state=1, PASS
cuVSLAM status: tracking healthy, PASS
Stop cuVSLAM: no matching container process, PASS
D435i Stop while tracking:
  cuVSLAM stopped -> bridge stopped -> Gazebo stopped, PASS
Final deployment:
  D435i topics ready
  cuVSLAM process running
  vo_state=1
```

## Tests Not Run

- IMU fusion was not enabled. The simulated optical-to-IMU transform and axis
  convention require a separate verified calibration change.
- No robot motion trajectory was executed; the current Gazebo model remains
  stationary.
- No map save/load, relocalization, tracking-loss injection, or long-duration
  soak test was run.
- No Nav2 integration was attempted.

## Residual Risk

- This result proves simulated stereo tracking, not real D435i accuracy.
- The static 5 cm stereo transform is derived from the declared SDF geometry.
- The host user systemd manager does not currently inherit the user's Docker
  group. cuVSLAM lifecycle therefore uses SSH plus detached `docker exec`.
- The container runtime log is `/tmp/cuvslam-d435i-vslam.log` and is replaced
  by the next launch.

## Rollback

1. Stop cuVSLAM from the GUI.
2. Remove the deployed simulation launch from the remote package.
3. Rebuild only `isaac_ros_visual_slam`.
4. Revert the cuVSLAM section in `tools/jetson_connection_web.py`.

