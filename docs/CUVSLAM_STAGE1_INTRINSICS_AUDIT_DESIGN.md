# cuVSLAM Stage 1 Simulated Intrinsics Audit

- Date: 2026-07-30
- Environment: Jetson simulation workspace, ROS domain 43
- Change class: C1 diagnostic

## Objective

Prove whether the Gazebo camera FOV and the `CameraInfo` consumed by cuVSLAM
describe the same rectified stereo model. Do not tune motion, TF, exposure, or
cuVSLAM parameters in this stage.

## Baseline

The active SDF declares both infrared cameras as `424x240`, with
`horizontal_fov=1.58825` radians and a 30 Hz update rate. A live left
`CameraInfo` sample reports `fx=208.33176612854004`, which implies
`horizontal_fov=1.588249876944` radians.

## Options

### A. Change the SDF FOV or CameraInfo values

This would be appropriate only if a measured mismatch exceeded the declared
tolerance. The observed FOV difference is approximately `1.23e-7` radians, so
changing either side would add an unsupported calibration error.

### B. Preserve the calibration and add an executable contract validator

Selected. The validator parses the SDF, samples the live left and normalized
right `CameraInfo`, and checks dimensions, FOV, K/P consistency, principal
point, rectification, zero distortion, left/right intrinsic equality, and the
right projection baseline.

## Acceptance

- Every scalar check passes with a `1e-5` tolerance.
- The normalized right projection satisfies `P[3] = -fx * 0.05 m`.
- cuVSLAM reaches tracking with the validated inputs.
- Gazebo, bridge, and cuVSLAM are stopped before the stage report.

## Rollback

Remove the validator, its unit test, and this design note. No runtime topic,
frame, launch, SDF, or real-robot workspace is changed.
