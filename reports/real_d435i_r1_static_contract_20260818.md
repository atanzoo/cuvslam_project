# Real D435i R1 Static Contract

Date: 2026-08-18 (Asia/Taipei)  
Environment: real Intel RealSense D435i on Jetson AGX Orin  
Experiment ID: `real_d435i_r1_20260818_1034`

## Result

R1 stereo and stationary-IMU data contract: **PASS with restrictions**.

The real D435i publishes synchronized rectified stereo at 848x480/30 Hz and a
combined IMU stream at 200 Hz. Factory CameraInfo contains a 0.050072 m stereo
baseline. Static IMU timing and stationary values pass the R1 gates. The first
real cuVSLAM run may proceed in stereo-only mode with
`enable_imu_fusion=false`.

IMU fusion remains blocked because librealsense reports that IMU calibration
is unavailable and substitutes default intrinsic/extrinsic values. A previous
Motion Module hardware error did not recur after an `initial_reset`, but this
does not remove the calibration restriction.

## Configuration

- Jetson Linux: R35.5.0, kernel 5.10.192-tegra.
- Isaac ROS Visual SLAM: 2.1.0 / cuVSLAM 11.4.
- RealSense ROS: 4.54.1.
- librealsense: 2.54.2.
- D435i serial: 261222077990.
- ASIC serial: 261143060940.
- Firmware: 5.15.1.55; reported recommended firmware 5.15.1.
- USB descriptor: 3.2.
- ROS domain: 42.
- Streams: infra1/infra2 848x480 Y8 at 30 Hz; gyro/accel 200 Hz;
  `unite_imu_method=2`; depth/color disabled.

The Jetson booted with its clock at 1970-01-01. It was manually synchronized
from the Mac before recording. `NTPSynchronized` remained false, so every
future real run must reject an invalid system date before starting ROS nodes.

## Measurements

The 58.65 s bag contains 18,741 messages and is 1.3 GiB.

| Measurement | Result |
|---|---:|
| Left images | 1,759 at 29.9925 Hz |
| Right images | 1,759 at 29.9925 Hz |
| Maximum image period | 33.411 ms |
| Stereo exact timestamp fraction | 1.000 |
| Maximum left/right timestamp skew | 0.000 ms |
| IMU messages | 11,704 at 199.529 Hz |
| Maximum IMU period | 5.023 ms |
| Non-monotonic image/IMU timestamps | 0 |
| IMU frame | `camera_imu_optical_frame` |
| Gyro norm P95, stationary | 0.01003 rad/s |
| Acceleration norm median | 9.58798 m/s^2 |
| Right CameraInfo P[3] | -21.450237 |
| Projected stereo baseline | 0.050072 m |

Both image topics are `mono8`, 848x480, with a step of 848 bytes. The left
image frame is `camera_infra1_optical_frame`; the right image frame is
`camera_infra2_optical_frame`. The right CameraInfo uses
`camera_infra1_optical_frame` as its projection reference and carries the
right-camera baseline in P[3], as implemented by the RealSense ROS wrapper.
No simulation CameraInfo adapter was used.

## QoS And TF

- Every required camera/IMU topic had exactly one publisher: `/camera/camera`.
- Infrared image publishers offered Reliable/Transient Local QoS.
- CameraInfo and IMU publishers offered Reliable/Volatile QoS.
- `/tf_static` had one publisher, `/camera/camera`; no dynamic `/tf` messages
  were present with `tf_publish_rate=0`.
- The static tree includes `camera_link`, both physical infrared frames, both
  optical frames, accel/gyro frames, and combined IMU frames.
- Factory physical right-camera translation is 0.050072 m from the left/base
  optical reference.

The live offered QoS is compatible with a Best Effort/Volatile sensor-data
subscriber, but the first cuVSLAM launch must verify that all four stereo
subscriptions actually receive messages before accepting tracking output.

## Resource Baseline

- Root storage after recording: 12 GiB free, 80% used.
- CPU temperature: approximately 42.4--42.7 C.
- GPU temperature: approximately 37.6 C.
- RAM usage: approximately 1.59 GiB of 62.8 GiB.

## Restrictions And Next Gate

1. Keep `enable_imu_fusion=false` for R2/R3.
2. Add a system-time preflight; reject years earlier than 2025.
3. Do not modify factory CameraInfo, stereo baseline, or RealSense TF based on
   this single run.
4. Do not use wheel odometry, Gazebo truth, the simulation adapter, Nav2, or
   motor control.
5. R2 must launch real stereo cuVSLAM, confirm four input subscriptions,
   confirm one owner for `map -> odom` and `odom -> base_link`, and run a
   stationary odometry test before any physical motion.

## Evidence

Small evidence is copied to
`logs/real_d435i_r1_20260818_1034/`. The complete bag remains on Jetson at:

`/home/tseng/isaac_ros_data/experiments/real_d435i_r1_20260818_1034/rosbag2`

