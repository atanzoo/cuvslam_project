# Mac GUI Start Simulation Fix

Date: 2026-07-31

## Symptom

The GUI reported that Gazebo and the five D435i topics were ready, but
cuVSLAM never reported `vo_state=1`. The container had image and IMU topic
publishers, but no live sensor messages were reaching the tracker.

## Causes

Two configuration defects were found:

1. The GUI world path omitted `share/slam_gazebo`, so Gazebo received a
   nonexistent dense v3 SDF path and attempted a Fuel download before exiting.
2. The native-pose bridge was always enabled. Its `Pose_V` message contains
   approximately 131 transforms and overloaded the live ROS data path. Route
   tests need this data only through the compact native-pose relay.

## Fix

- GUI now uses:

  ```text
  /home/tseng/jetson_slam_ws/install/slam_gazebo/share/slam_gazebo/worlds/cuvslam_mapping_observable_640x360_range2p5_dense_turn.sdf
  ```

- The GUI checks that the world file exists before starting services.
- Native pose bridge is opt-in with `enable_native_pose:=true`; the GUI
  leaves it disabled.
- The mapping route runner explicitly enables native pose and uses the compact
  relay.
- GUI cuVSLAM launch parameters are stereo-only, mapping enabled, sparse
  visualization enabled, observations enabled, and IMU fusion disabled.
- Monitor no longer treats wheel odometry or the evaluation-only world outline
  as a live tracking prerequisite.

## Verification

After deploying the SDF and bridge fix, the same live launch produced:

- Gazebo service active;
- image and IMU topics present and connected to cuVSLAM;
- cuVSLAM status with `vo_state=1`;
- cuVSLAM log confirming `Enable IMU Fusion: false`;
- no native Pose_V bridge in the normal GUI path.

The current live runtime is left active for Mac Foxglove inspection.
