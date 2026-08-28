# cuVSLAM Stage 4 Official Input Contract Design

Date: 2026-07-30  
Change class: C2 launch parameter; C3 estimator experiment  
Deployment target: simulation workspace only

## Evidence Before Change

The installed Isaac ROS 2.1 D435i launch enables IMU fusion, uses the infrared
stereo pair, and supplies `camera_gyro_optical_frame` as the IMU input frame.
The installed Isaac Sim launch demonstrates that rectified stereo without IMU
is supported, so the current stereo-only simulation is valid but does not match
the complete D435i example.

The simulated IMU publishes at 200 Hz and its static acceleration is consistent
with the Gazebo model's native 6DoF pitch. The current TF tree connects the
message frame `slam_bot/camera_imu_frame/d435i_imu` to the left camera and
`base_link`.

## Options

### Option A: Keep stereo-only

Advantages:

- preserves the existing baseline;
- isolates visual geometry;
- matches NVIDIA's Isaac Sim stereo-only architecture.

Disadvantages:

- does not match the official D435i VIO data path;
- leaves rotation and gravity constraints unused.

Compatibility: no interface change.

### Option B: Add opt-in IMU fusion and run an A/B trial

Advantages:

- matches the official D435i estimator path more closely;
- uses the validated 200 Hz simulated IMU;
- keeps the existing stereo-only baseline available.

Disadvantages:

- introduces IMU timing, noise, and axis assumptions;
- can regress if the simulated IMU contract differs from RealSense.

Compatibility: adds a launch argument while retaining `false` as the default.

## Decision

Select Option B for a controlled simulation-only A/B trial. Set
`input_imu_frame` to the exact `/d435i/imu` message frame and change no camera,
TF, world, rate, or cuVSLAM noise parameter. Accept the variant only if tracking
remains healthy and native-6DoF translation and rotation residuals improve.
Otherwise retain the stereo-only default.

## Rollback

Launch with `enable_imu_fusion:=false`, which is the default, or revert the
launch argument addition. No TF owner or topic contract changes.
