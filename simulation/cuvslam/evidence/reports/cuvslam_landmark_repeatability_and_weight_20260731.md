# cuVSLAM Landmark Repeatability and Weight Diagnosis

Date: 2026-07-31 (Asia/Taipei)  
Environment: Jetson AGX Orin, ROS 2 Humble container, ROS domain 43  
World: `cuvslam_mapping_observable.sdf`  
Motion: independent cold-start 1 m straight trials  
IMU fusion: disabled

## Decision

The sparse landmark geometry failure is repeatable and predominantly
systematic. A third visualization-only diagnostic proved that the cuVSLAM
`LL_MAP` landmark weight is `1.0` for every exported point, including points
several metres from any known SDF surface. Map-layer weight cannot be used to
separate valid and invalid geometry.

Do not proceed to the 90-degree turn gate.

## Bundles

```text
Run 1:
/home/tseng/isaac_ros_ws/data/straight_mapping_observable_20260731_35497

Run 2:
/home/tseng/isaac_ros_ws/data/straight_mapping_observable_repeat2_20260731_38892

Weight diagnostic:
/home/tseng/isaac_ros_ws/data/straight_mapping_weight_diagnostic_20260731_42742
```

## Trajectory comparison

All trajectory metrics use first-pose rigid SE(3) alignment and fixed scale.
No Sim(3) correction is applied.

| Metric | Run 1 | Run 2 | Weight diagnostic |
|---|---:|---:|---:|
| Truth distance | 1.0140 m | 1.0104 m | 1.0134 m |
| cuVSLAM distance | 0.9548 m | 0.9321 m | 0.9528 m |
| Rotation | 1.46 deg | 1.31 deg | 1.58 deg |
| Translation ATE RMSE | 0.0495 m | 0.0697 m | 0.0639 m |
| Translation ATE P95 | 0.0746 m | 0.1081 m | 0.0974 m |
| 1 s translation RPE RMSE | 0.0101 m | 0.0140 m | 0.0131 m |
| Status | 581 / 581 state 1 | 579 / 579 state 1 | 507 / 507 state 1 |

Run 2 narrowly failed the provisional 0.10 m translation ATE P95 gate.
Tracking remains stable, but straight trajectory accuracy is near the gate
rather than comfortably below it.

## Landmark-to-SDF geometry

| Metric | Run 1 | Run 2 | Weight diagnostic |
|---|---:|---:|---:|
| Final landmarks | 379 | 378 | 387 |
| Surface-distance median | 0.2430 m | 0.3159 m | 0.3191 m |
| Surface-distance P90 | 1.7618 m | 1.8473 m | 1.9069 m |
| Surface-distance maximum | 5.0750 m | 6.7848 m | 6.3730 m |
| Within 0.10 m | 35.9% | 35.2% | 36.2% |
| Within 0.25 m | 50.1% | 47.9% | 47.8% |
| Over 0.25 m | 49.9% | 52.1% | 52.2% |

All three runs fail the provisional requirement of at least 80 percent within
0.25 m and no more than 20 percent over 0.25 m.

## Error growth

The geometry error exists in the first exported snapshot and becomes worse as
the map accumulates:

| Run | Snapshot 1 | Snapshot 2 | Snapshot 3 |
|---|---:|---:|---:|
| Run 1 median | 0.1670 m | 0.2210 m | 0.2430 m |
| Run 1 within 0.25 m | 54.0% | 52.2% | 50.1% |
| Run 2 median | 0.2055 m | 0.3176 m | 0.3159 m |
| Run 2 within 0.25 m | 52.7% | 48.4% | 47.9% |
| Weight diagnostic median | 0.1785 m | 0.2430 m | 0.3191 m |
| Weight diagnostic within 0.25 m | 53.6% | 50.2% | 47.8% |

This is not a late-run catastrophic event. Incorrect geometry is present from
the first map export, and later additions do not improve it.

## Cross-run repeatability

Final Run 1 versus Run 2 nearest-neighbour comparison:

- approximately 45 percent of points match within 1 mm;
- approximately 48 percent match within 1 cm;
- approximately 66 percent match within 5 cm;
- Run 1 to Run 2: 91.6 percent within 25 cm;
- Run 2 to Run 1: 88.1 percent within 25 cm.

The same scene repeatedly generates much of the same landmark structure,
including its incorrect geometry. The failure is therefore predominantly
systematic rather than random.

## NVIDIA wrapper evidence

The installed Isaac ROS 2.1 wrapper reads the `LL_MAP` data layer. Its
`CUVSLAM_LandmarkInfo` contains `id`, XYZ, and `weight`, but the standard ROS
message publishes only:

```text
x, y, z, rgb
```

For `LL_MAP`, `rgb` is a deterministic grayscale value generated from the
landmark id. It is not confidence.

A bounded diagnostic patch temporarily added the internal weight as a
PointCloud2 field. All 387 final landmarks reported:

```text
minimum = median = P90 = maximum = 1.0
```

Correct and incorrect landmarks had the same weight. Pearson correlation is
undefined because the weight has zero variance.

The patch was reversed after the diagnostic, and
`isaac_ros_visual_slam` was clean-first rebuilt. The standard PointCloud
schema is restored.

## Interpretation

Current evidence excludes:

- a one-off random mapping failure;
- a small map-to-world alignment error;
- a single global scale correction;
- filtering the `LL_MAP` cloud by exported weight.

The remaining leading boundary is between current tracked observations and
the accumulated `LL_MAP` geometry. The next diagnostic should enable and
record `/visual_slam/vis/observations_cloud`, compare observation depth with
the corresponding SDF surfaces, and measure stereo disparity at those
observations. This distinguishes:

1. incorrect stereo triangulation before map insertion;
2. correct current observations but incorrect accumulated-map semantics or
   optimization;
3. a visualization/export interpretation defect.

Do not clip out-of-arena points or apply truth-assisted filtering to pass the
map gate.

