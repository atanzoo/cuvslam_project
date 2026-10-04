# cuVSLAM Straight Mapping Report

Date: 2026-07-29
Environment: Jetson AGX Orin simulation, ROS 2 Humble domain 43,
cuVSLAM 11.4 stereo-only

## Objective

Test whether the corrected right CameraInfo pipeline can produce a stable
local visual map during straight-only movement, before returning to the known
turning failure.

Ground truth was recorded for offline evaluation only. The controller
subscribed only to `/scan` and `/visual_slam/status`.

Experiment root:

```text
/home/tseng/isaac_ros_ws/data/experiments/straight_mapping_20260729_1025
```

## Runtime Configuration

- right projection baseline: `P[3]=-10.416588`;
- derived stereo baseline: `0.05 m`;
- localization and mapping: enabled;
- SLAM visualization and sparse landmarks: temporarily enabled;
- observations and other high-volume debug views: disabled;
- IMU fusion: disabled.

## Results

| Stage | Truth 2D/3D | cuVSLAM 2D | cuVSLAM 3D | Z | Rotation |
|---|---:|---:|---:|---:|---:|
| Short, 3 s | `0.2475 m` | `0.3827 m` | `0.3828 m` | `-0.0018 m` | `5.66 deg` |
| Medium, 6 s | `0.4375 m` | `0.2207 m` | `0.2524 m` | `-0.1225 m` | `13.80 deg` |

All status samples remained `vo_state=1`. No tracking-loss, frame-drop, or
image-jitter warning appeared in the launch log.

The short run overestimated displacement by about 55 percent, while the
medium run underestimated 3D displacement by about 42 percent. The remaining
error is therefore not a stable multiplicative stereo-scale factor.

## Sparse Map Evidence

| Stage | Messages | Non-empty | Final finite points | Final XYZ bounds |
|---|---:|---:|---:|---|
| Short | 10 | 9 | 583 | X `[0.7626, 10.8520]`, Y `[0.4102, 8.2194]`, Z `[-1.2576, 1.4645]` |
| Medium | 9 | 9 | 460 | X `[0.7394, 10.7774]`, Y `[0.4102, 8.7814]`, Z `[-1.5921, 1.4645]` |

The arena is approximately 6 m by 6 m. Landmark bounds beyond 10 m and the
large vertical span are inconsistent with the known world. The map topic is
functional, but the geometry is not trustworthy.

## Decision

The 1 m and 2 m stages were not run after the medium quality gate failed. The
controller published zero velocity, and sparse visualization was returned to
its lower-load disabled state.

This experiment does not invalidate the corrected right CameraInfo contract.
It shows that another camera-motion or estimator-input inconsistency remains.
The next useful isolation is measured image optical flow versus Gazebo
ground-truth translation, declared focal length, and frame timestamps.
