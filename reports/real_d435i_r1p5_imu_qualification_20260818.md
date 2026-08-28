# Real D435i R1.5 IMU Qualification

Date: 2026-08-18 (Asia/Taipei)  
Device: Intel RealSense D435I, serial `261222077990`  
Firmware: `5.15.1.55`  
Jetson: AGX Orin, JetPack 5.1.3 / L4T R35.5.0  
Runtime: Isaac ROS Humble container, cuVSLAM 11.4, RealSense ROS 4.54.1, librealsense 2.54.2

## Result

R1.5 core fusion readiness: **PASS with one physical test pending**.

The IMU is usable by the real cuVSLAM pipeline. The remaining item before
declaring the full R1.5 qualification is the operator-performed X/Y/Z axis and
sign test; it cannot be inferred safely while the camera is stationary.

## Evidence

| Check | Result | Evidence |
|---|---|---|
| Calibration source | PASS with limitation | Direct public librealsense C++ API returned motion intrinsic and extrinsic records. Motion scale/cross-axis is identity, but bias/noise variances are all zero; this explains the ROS warning. |
| IMU-camera extrinsic | PASS | IMU-to-IR1 rotation is identity; translation magnitude is about 14.0 mm. ROS `/tf_static` exposes the corresponding camera/gyro/IMU frame tree. |
| Stationary IMU data | PASS | R1 bag: 11,704 samples at 199.529 Hz; timestamps monotonic; gyro P95 norm 0.01003 rad/s; acceleration norm median 9.588 m/s². |
| Static noise estimate | PASS | Gyro component standard deviation: `[0.00264, 0.00287, 0.00163]` rad/s; accel component standard deviation: `[0.01319, 0.01329, 0.01448]` m/s². |
| Camera/IMU timing | PASS | Stereo skew is exactly 0 ms in the R1 bag; nearest IMU-to-left-image skew p95 is 2.378 ms, max 2.503 ms. |
| Frame/convention | PASS for smoke test | Raw `/camera/imu` frame is `camera_imu_optical_frame`; the launch uses NVIDIA's RealSense cuVSLAM input frame `camera_gyro_optical_frame`, with a valid static TF path. |
| cuVSLAM fusion | PASS | `Enable IMU Fusion: true`; baseline read as 0.050072 m; `vo_state=1` for all 1,168 status messages in a 45.0 s stationary run. |
| Static fusion stability | PASS | Max position displacement 3.292 mm; max orientation change 0.00122 rad; no pose jump observed. |
| X/Y/Z dynamic axis/sign | PENDING | The first guided X-axis attempt was rejected: peak gyro was only about 0.014 rad/s, comparable to the stationary baseline, and the two directions did not show a sign reversal. A deliberate larger hand rotation is required. |

The ROS message covariance is currently diagonal `0.01` for angular velocity
and linear acceleration, with orientation covariance `-1` (orientation is not
provided by the sensor). The cuVSLAM launch retains NVIDIA's D435i contract:

```text
gyro_noise_density = 0.000244
gyro_random_walk   = 0.000019393
accel_noise_density = 0.001862
accel_random_walk   = 0.003
calibration_frequency = 200.0
```

## Launch and tools added

- Real hardware launch: [`isaac_ros_visual_slam_d435i_real.launch.py`](/Users/tsengpochien/Desktop/cuvslam_project/deployment/isaac_ros/isaac_ros_visual_slam_d435i_real.launch.py)
- Direct calibration audit: [`inspect_real_d435i_calibration.cpp`](/Users/tsengpochien/Desktop/cuvslam_project/tools/inspect_real_d435i_calibration.cpp)
- R1 bag evaluator with IMU noise/covariance/timing: [`evaluate_real_d435i_r1_bag.py`](/Users/tsengpochien/Desktop/cuvslam_project/tools/evaluate_real_d435i_r1_bag.py)
- Same-container stationary collector: [`collect_cuvslam_r1p5_smoke.py`](/Users/tsengpochien/Desktop/cuvslam_project/tools/collect_cuvslam_r1p5_smoke.py)

The launch was rebuilt successfully with the Jetson CUDA 11.4 mounts and is
installed at the `isaac_ros_visual_slam` package share, so it can be invoked by
package name in the next run.

For the operator-controlled dynamic test, a macOS GUI is available at
[Real D435i R1.5 Axis Test.command](</Users/tsengpochien/Desktop/cuvslam_project/Real D435i R1.5 Axis Test.command>).
It opens a browser GUI backed by
[`real_d435i_axis_web_gui.py`](</Users/tsengpochien/Desktop/cuvslam_project/tools/real_d435i_axis_web_gui.py>).
The GUI starts/stops the Jetson rig, runs one phase per button press, shows
live gyro bars and dominant axis, and downloads each phase JSON into
`logs/real_d435i_r1p5_gui/`. The GUI start/stop API was dry-run verified on
the real Jetson/D435i connection; the temporary container was stopped after
verification.

The original R1 bag remains the raw camera/IMU evidence. The Jetson-side
fusion metrics are under:

```text
/home/tseng/isaac_ros_data/experiments/real_d435i_r1p5_20260818_1100/
```

## Important operational restriction

Jetson time was set correctly for this run, but `NTPSynchronized=no` remains.
Before every formal run, perform a time preflight and set time from the Mac or
an available NTP source if the Jetson has booted without a valid RTC/network.

## Next action

Keep the D435i connected and run the prepared dynamic test while recording only
the IMU. Perform separate slow rotations around the camera optical X, Y, and Z
axes, then repeat each direction with the opposite sign. The test is complete
when each commanded axis produces a dominant raw gyro response on the expected
axis and the sign is consistent with the chosen ROS optical-frame convention.
