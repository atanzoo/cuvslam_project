# cuVSLAM Triangulation Isolation And First Round Trip

Date: 2026-07-31 (Asia/Taipei)

## Decision

The minimum test confirmed a stereo-triangulation failure before `LL_MAP`
accumulation. A simulation-only combination of `640x360 @ 30 Hz` and a
`2.5 m` stereo rendering range improved current-observation geometry enough
to pass the provisional minimum gate. The subsequent 1 m out-and-back run did
not pass the trajectory or final-map gate.

Do not begin a turn, square route, map save/load claim, or Nav2 integration
from this result.

## Controlled contract

The following remained unchanged in every A/B:

- cuVSLAM 11.4 / Isaac ROS Visual SLAM release 2.1;
- 30 Hz camera update rate;
- 5 cm stereo baseline;
- horizontal FOV `1.58825 rad`;
- rectified stereo topics and normalized right CameraInfo;
- camera/base TF contract;
- IMU fusion disabled;
- robot speed `0.12 m/s`;
- observable-world collision geometry.

`/visual_slam/vis/observations_cloud` was enabled only for diagnostics.

## Minimum-test result

The initial `424x240 @ 30 Hz` 0.25 m run produced:

| Metric | Result |
|---|---:|
| Analyzed points | 5,688 |
| Surface-distance median | 0.4077 m |
| Surface-distance P90 | 2.3465 m |
| Within 0.25 m | 43.8% |
| Maximum truth synchronization skew | 25 ms |

The declared `camera_infra1_frame` interpretation produced 43.8% within
0.25 m; an optical-frame counterfactual produced only 5.1%. This excluded an
exported-cloud frame misunderstanding.

Depth stratification localized the failure:

| Forward depth | Expected disparity | Within 0.25 m |
|---|---:|---:|
| 0–1.5 m | 7.75 px | 68.5% |
| 1.5–2.5 m | 6.00 px | 76.5% |
| 2.5–4.0 m | 2.94 px | 26.4% |
| 4.0 m and beyond | 2.23 px | 0.0% |

Bad geometry was already present in current observations. It was not created
solely by `LL_MAP`, map-frame conversion, or backend loop closure.

## Repair experiments

All rows are independent cold-start 0.25 m runs.

| Variant | Observation messages | Within 0.25 m | P90 | Decision |
|---|---:|---:|---:|---|
| 424x240, original range | 307 | 43.8% | 2.3465 m | Fail |
| 640x360, original range | 234 | 42.2% | 1.7200 m | Fail |
| 424x240, 2.5 m range | 361 | 78.1% | 0.6052 m | Improved, below gate |
| 424x240, 2.0 m range | 414 | 72.4% | 0.7157 m | Fail; catastrophic false depths |
| 424x240, 2.5 m, extra patches | 332 | 60.8% | 0.7436 m | Fail; matching worsened |
| 640x360, 2.5 m range | 200 | **87.2%** | **0.3986 m** | Minimum gate pass |

The accepted minimum-test variant used focal length `314.462986 px`, baseline
`0.050000 m`, and had a maximum truth synchronization skew of `25 ms`.

The range setting is not an estimator-side disparity filter. False
correspondences can still triangulate beyond the render range. It is a
simulation input-conditioning measure, not a real D435i accuracy claim.

## First out-and-back run

Bundle:

```text
/home/tseng/isaac_ros_ws/data/roundtrip_mapping_1m_640x360_range2p5_20260731_56955
```

Motion and health:

- outbound truth distance `1.0104 m`;
- final truth distance from start `0.0150 m`;
- no intentional orientation change;
- all 795 status samples reported `vo_state=1`.

### Trajectory

| Metric | Result |
|---|---:|
| cuVSLAM odometry closure displacement | 0.1834 m |
| cuVSLAM odometry closure rotation | 6.07 deg |
| SLAM-path closure displacement | 0.2137 m |
| SLAM-path closure rotation | 6.46 deg |
| Translation ATE RMSE / P95 | 0.1644 / 0.2045 m |
| Rotation ATE RMSE / P95 | 4.55 / 6.08 deg |
| 1 s translation RPE RMSE / P95 | 0.0406 / 0.0840 m |
| 1 s rotation RPE RMSE / P95 | 1.12 / 2.24 deg |

Tracking health therefore did not imply geometric closure.

### Current observations

Sixty sampled clouds from the route contained 6,980 points:

- 84.4% within 0.25 m;
- median `0.0465 m`;
- P90 `0.5640 m`;
- maximum `12.7122 m`;
- no clouds rejected by the 100 ms synchronization gate.

Most in-range points were good, but false correspondences still generated
148 points beyond 4 m despite the 2.5 m render range.

### Final landmarks

The final `LL_MAP` cloud contained 1,182 landmarks:

| Metric | Result |
|---|---:|
| Surface-distance median | 0.0507 m |
| Surface-distance P90 / P95 | 0.6129 / 0.7634 m |
| Within 0.10 m | 62.9% |
| Within 0.25 m | 78.4% |
| Over 0.25 m | 21.6% |
| Maximum error | 12.5851 m |

Map snapshots improved from 66.7% within 0.25 m at 81 points to 78.4% at
1,182 points. Re-observation and backend accumulation improved the map, but
did not remove enough wrong landmarks during this route.

## Architecture interpretation

This behavior matches the separation in NVIDIA's release-2.1 architecture:

```text
stereo images
  -> current LL_OBSERVATIONS: improved to 84.4%
  -> tracking / pose: still failed closure
  -> accumulated LL_MAP: improved over time, ended at 78.4%
```

The remaining failure is not merely a point-cloud display issue. It combines
residual false stereo correspondences with insufficient rejection/maturation
for this short forward/reverse route.

Official references:

- <https://nvidia-isaac-ros.github.io/v/release-2.1/concepts/visual_slam/cuvslam/index.html>
- <https://nvidia-isaac-ros.github.io/v/release-2.1/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html>

## Next gate

Do not add a turn yet. The next change must act at the correspondence or
landmark-acceptance boundary rather than adding more scene texture.

Preferred order:

1. determine whether a newer supported cuVSLAM/Isaac ROS interface exposes
   minimum disparity, maximum triangulation depth, landmark age, or
   observation-count filtering;
2. if release 2.1 must remain fixed, build an external repeated-observation map
   product with explicit depth and multi-view support gates, retaining raw
   `LL_MAP` as evidence;
3. rerun this exact out-and-back test and require trajectory closure at most
   `0.10 m` / `3 deg`, plus at least 85% final landmarks within `0.25 m`,
   P90 at most `0.25 m`, and no catastrophic multi-meter points.
