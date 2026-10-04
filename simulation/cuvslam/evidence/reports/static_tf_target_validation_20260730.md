# Static TF Target Validation

Date: 2026-07-30  
Environment: Jetson AGX Orin simulation, ROS domain 43  
World: Baseline `indoor_gz_sim.sdf`  
Result: PASS, three independent runs

## Purpose

Validate the `base_link -> sensors` Frame Contract independently from cuVSLAM
motion estimation.

The deployed cuVSLAM process was stopped during each validator run so no
authoritative or validation TF edge had duplicate ownership. The production
launch and SDF were not modified.

## Absolute Targets

The validator parsed these poses directly from the SDF:

| Target | World position (m) | World yaw |
|---|---|---:|
| `feature_box_a` | `(1.4, 1.1, 0.35)` | `+0.35 rad` |
| `feature_box_b` | `(-1.6, -0.9, 0.30)` | `-0.60 rad` |
| `ceiling_light` | `(0, 0, 4.0)` | `0 rad` |

The Baseline robot starts at `(0,0,0.06)` with zero rotation.

Example theoretical coordinates in the left optical frame:

| Target | Left-optical XYZ (m) |
|---|---|
| `feature_box_a` | `(-1.075, -0.090, +1.210)` |
| `feature_box_b` | `(+0.925, -0.040, -1.790)` |
| `ceiling_light` | `(+0.025, -3.740, -0.190)` |

These values exercise optical right/down/forward signs, targets in front of
and behind the camera, and a large vertical displacement.

## Frames

Each target was evaluated in:

```text
base_link
camera_link
camera_infra1_frame
camera_infra1_optical_frame
camera_infra2_frame
camera_infra2_optical_frame
camera_imu_frame
lidar_link
```

This produced 24 comparisons per run.

## Method

The SDF parser independently computed:

```text
expected(frame -> target)
  = inverse(SDF world -> frame) * (SDF world -> target)
```

The ROS validator published the Frame Contract candidate edges and queried:

```text
observed(frame -> target) = tf2 lookup(frame, target)
```

Residual:

```text
inverse(expected) * observed
```

Default acceptance:

- translation residual `<=0.01 m`;
- rotation residual `<=0.5 deg`;
- optical baseline error `<=0.001 m`;
- all 24 lookups present.

## Results

Experiment directory:

```text
/home/tseng/isaac_ros_ws/data/experiments/
  static_tf_target_validation_20260730_101214
```

| Trial | Observed | Max translation error | Max rotation error | Baseline error | Failures |
|---|---:|---:|---:|---:|---:|
| 1 | 24/24 | `0.000000000 m` | `0.000000000 deg` | `0.000000000 m` | 0 |
| 2 | 24/24 | `0.000000000 m` | `0.000000000 deg` | `0.000000000 m` | 0 |
| 3 | 24/24 | `0.000000000 m` | `0.000000000 deg` | `0.000000000 m` | 0 |

Local verification also passed six tests covering SDF parsing, edge
composition, transform inversion, and optical-axis semantics.

## Decision

The following static geometry is accepted and locked for the current
simulation SDF:

```text
base_link -> camera_link
camera_link -> camera_infra1_frame
camera_infra1_frame -> camera_infra1_optical_frame
camera_link -> camera_infra2_frame
camera_infra2_frame -> camera_infra2_optical_frame
camera_link -> camera_imu_frame
base_link -> lidar_link
```

Do not tune these values to compensate for cuVSLAM trajectory errors. Any
future change requires a changed SDF/calibration source and rerunning this
three-target test.

This result validates static geometry only. The current production launch was
restored after the test and still publishes
`odom -> camera_infra1_optical_frame`; therefore the complete locked static
tree is not yet active. Dynamic frame ownership and cuVSLAM motion accuracy
remain open issues.

After validation, the original cuVSLAM runtime was restarted and returned to
`vo_state=1`.
