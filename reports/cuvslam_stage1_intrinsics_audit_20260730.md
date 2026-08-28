# cuVSLAM Stage 1 Intrinsics Audit

Date: 2026-07-30  
Environment: Jetson AGX Orin simulation, ROS domain 43  
Result: Camera FOV and K/P contract passed; trajectory defect remains  
Final runtime state: Simulation stopped

## Scope

This stage tested only:

- Gazebo's declared infrared camera FOV and image dimensions;
- live left `CameraInfo`;
- the normalized right `CameraInfo` consumed by cuVSLAM;
- cuVSLAM tracking readiness after validation.

It did not change the SDF, TF, launch parameters, cuVSLAM algorithm, timing,
exposure, motion sampling, or the real-robot workspace.

## Static Model

Both `d435i_infra1` and `d435i_infra2` declare:

```text
width=424
height=240
horizontal_fov=1.58825 rad
update_rate=30 Hz
format=L8
```

## Live Contract Result

The validator sampled `/d435i/infra1/camera_info` and
`/cuvslam/input/infra2/camera_info`.

```text
left fx=208.33176612854004 px
right fx=208.33176612854004 px
CameraInfo-derived horizontal FOV=1.58824987681 rad
SDF horizontal FOV=1.58825 rad
absolute FOV error=1.23e-7 rad
```

All dimension, K/P, principal-point, rectification, distortion, and
left/right intrinsic checks passed with a `1e-5` scalar tolerance.

The normalized right projection also passed:

```text
actual P[3]=-10.4165883064
expected -fx * 0.05m=-10.4165883064
error=0
```

The validator summary was:

```text
SUMMARY status=PASS failures=0
```

cuVSLAM reported:

```text
vo_state: 1
track_execution_time_mean: 0.004691748 s
```

## Minimal Change

No calibration value was changed because the measured model is already
consistent. The C1 correction adds a repeatable executable validator and four
unit tests:

- `tools/validate_sim_camera_intrinsics.py`;
- `tools/test_validate_sim_camera_intrinsics.py`;
- `docs/CUVSLAM_STAGE1_INTRINSICS_AUDIT_DESIGN.md`.

Local result:

```text
Ran 4 tests in 0.003s
OK
```

## Motion Smoke Test

A short forward command was used only to confirm that the existing trajectory
defect remains observable after adding the diagnostic. Sequential endpoint
samples were:

```text
ground-truth displacement=0.50575 m
cuVSLAM horizontal displacement=0.76379 m
cuVSLAM 3D displacement=0.76429 m
cuVSLAM false rotation=24.86 deg
vo_state=1
```

This is not a formal accuracy result: the endpoints were sampled sequentially,
not from a timestamp-aligned rosbag, and command duration included ROS CLI
startup/shutdown behavior. It must not replace the prior formal Trial 1 result
of `22.4%` horizontal distance error. It only shows that validating FOV and K/P
did not remove the motion failure.

## Stage Decision

The hypothesis that a Gazebo FOV versus CameraInfo K/P mismatch is the current
root cause is rejected. The active values agree far inside tolerance, so
tuning them would introduce an unsupported calibration error.

Proceed next to Stage 2: measure true left/right image disparity and epipolar
geometry. Do not alter TF or intrinsics during that stage.

## Final Shutdown Evidence

After testing:

```text
cuvslam-d435i-gazebo.service: inactive/dead
cuvslam-d435i-bridge.service: inactive/dead
cuVSLAM launch PID count: 0
ROS domain 43 topics: /parameter_events, /rosout
```
