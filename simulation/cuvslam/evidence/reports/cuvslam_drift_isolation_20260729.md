# cuVSLAM Drift Isolation Report

Date: 2026-07-29
Environment: Jetson AGX Orin simulation, ROS 2 Humble, ROS domain 43,
cuVSLAM 11.4 stereo-only

## Scope

Determine when the simulated planar trajectory first separates from the
ground-truth plane. Ground truth was recorded for offline evaluation only. The
motion controller subscribed only to `/scan` and `/visual_slam/status`.

Experiment root:

```text
/home/tseng/isaac_ros_ws/data/experiments/drift_isolation_20260729_0905
```

## Baseline Isolation

| Profile | Truth | cuVSLAM | Result |
|---|---|---|---|
| Static | zero motion | zero motion | no stationary drift |
| Straight | 0.2355 m, Z 0, rotation 0 | 0.3779 m, Z +0.1112 m, rotation 5.59 deg | error begins during translation |
| Turn | 37.70 deg, zero translation | 12.47 deg, 1.2963 m translation, Z +0.1578 m | rotation strongly amplifies error |

All samples reported `vo_state=1`. Tracking state alone therefore did not prove
trajectory accuracy.

## Stereo Evidence

- 424x240 mono8 images.
- Left/right timestamps matched exactly.
- First-pair global shift: horizontal `+7 px`, vertical `0 px`.
- Camera intrinsics matched.
- Both CameraInfo projection matrices had `P[3]=0`.
- Stereo baseline was supplied only by TF.

The images were synchronized, vertically rectified, and non-empty. The missing
CameraInfo baseline remains a calibration-contract concern.

## Physical-Frame Hypothesis Test

The installed source and bundled RealSense launch use non-optical
`input_left_camera_frame` and `input_right_camera_frame`. Option A in
`docs/CUVSLAM_SIM_CAMERA_FRAME_FIX_DESIGN.md` added the physical camera tree and
configured cuVSLAM with those frames.

Experiment root:

```text
/home/tseng/isaac_ros_ws/data/experiments/frame_fix_20260729_0925
```

Static remained perfect, but the first straight-motion gate regressed:

| Metric | Before | Option A |
|---|---:|---:|
| Truth distance | 0.2355 m | 0.2265 m |
| cuVSLAM 3D distance | 0.3779 m | 0.5792 m |
| Absolute Z change | 0.1112 m | 0.1567 m |
| False rotation | 5.59 deg | 11.79 deg |

The turn profile was not run after this failure. `/cmd_vel=0` was published and
the launch was rolled back.

## Current Conclusion

The plane separation is a real estimator/calibration error, not a Foxglove
display artifact and not stationary drift. Supplying the physical frames in
the most direct interpretation of the old API did not fix it.

## Right CameraInfo Baseline Result

The approved Option B added a simulation-only sensor adapter:

```text
/d435i/infra2/camera_info
  -> right_camera_info_adapter
  -> /cuvslam/input/infra2/camera_info
  -> cuVSLAM
```

The adapter preserves the raw topic and sets only
`P[3] = -P[0] * 0.05`. Runtime values were:

```text
P[0] = 208.331766
P[3] = -10.416588
derived baseline = 0.050000 m
```

cuVSLAM logged that it read `left_pose_right` from CameraInfo. The explicit
stereo TF input was disabled.

Experiment root:

```text
/home/tseng/isaac_ros_ws/data/experiments/camera_info_baseline_20260729_0955
```

| Profile | Truth | Corrected CameraInfo cuVSLAM |
|---|---|---|
| Static | zero motion | zero motion |
| Straight | `0.2270 m` | `0.2792 m`, Z `-0.0626 m`, rotation `6.75 deg` |
| Turn | `36.73 deg`, zero translation | `9.04 deg`, `1.1374 m` translation, Z `-0.1974 m` |

The correction substantially improved straight scale and Z error, but did not
pass the rotation-quality gate. The right CameraInfo contract is corrected and
retained; cuVSLAM mapping accuracy remains unaccepted.

The next diagnosis should compare measured image angular flow against Gazebo
ground-truth yaw and declared focal length. Planar constraint or IMU fusion
must not be used to hide this remaining camera-motion inconsistency.
