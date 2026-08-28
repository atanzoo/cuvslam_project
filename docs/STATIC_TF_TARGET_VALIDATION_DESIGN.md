# Static TF Target Validation

Date: 2026-07-30  
Status: Passed three isolated runs; static geometry locked  
Change class: C1 diagnostic  
Runtime impact: none; the production cuVSLAM launch is not modified

## Objective

Validate the complete `base_link -> sensors` static Frame Contract against
independent SDF geometry by projecting multiple absolute stationary targets
into every sensor frame.

## Isolation

The validator runs only while cuVSLAM is stopped. It publishes a
validation-only root named `simulation_validation_world`, the candidate static
sensor tree, and three target frames. It exits after the comparison.

Gazebo motion, cuVSLAM parameters, CameraInfo, and the deployed launch are not
changed.

## Truth Source

The checker parses
`cuvslam_sim_observability/worlds/indoor_gz_sim.sdf` directly.

Targets:

| Target | Absolute SDF pose |
|---|---|
| `feature_box_a` | `(1.4, 1.1, 0.35, 0, 0, 0.35)` |
| `feature_box_b` | `(-1.6, -0.9, 0.30, 0, 0, -0.60)` |
| `ceiling_light` | `(0, 0, 4.0, 0, 0, 0)` |

These targets are asymmetric, non-collinear, and have different heights.

## Frames Under Test

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

For every target and frame:

```text
expected = inverse(SDF world -> frame) * (SDF world -> target)
observed = TF lookup(frame, target)
```

Translation and rotation residuals are calculated from:

```text
residual = inverse(expected) * observed
```

## Acceptance

Run three independent validator processes. Every run must pass:

- all 24 target/frame comparisons exist;
- maximum translation residual `<= 0.01 m`;
- maximum rotation residual `<= 0.5 deg`;
- left/right optical baseline is `0.050 m` within `0.001 m`;
- body forward maps to optical `+Z`;
- body left maps to optical `-X`;
- body up maps to optical `-Y`;
- no duplicate publisher owns a tested edge.

The production static tree may be locked only after all three runs pass.
Dynamic `map -> odom -> base_link` remains a separate estimator acceptance.

## Result

Three independent validator processes passed:

| Trial | Comparisons | Max translation error | Max rotation error | Baseline error |
|---|---:|---:|---:|---:|
| 1 | 24/24 | `0.000000000 m` | `0.000000000 deg` | `0.000000000 m` |
| 2 | 24/24 | `0.000000000 m` | `0.000000000 deg` | `0.000000000 m` |
| 3 | 24/24 | `0.000000000 m` | `0.000000000 deg` | `0.000000000 m` |

The contract values for `base_link -> sensors` are now locked against this SDF
version. This locks the geometry, not the current runtime topology: the rolled
back production cuVSLAM launch still does not publish the complete static tree.
Activating the locked tree requires resolving the dynamic child-frame
ownership without changing these static numeric values.

## Rollback

The validator is a temporary process. Stop it or let it exit. Because it does
not edit the deployed launch, no runtime rollback is required.
