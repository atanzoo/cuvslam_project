# cuVSLAM Controlled 90-Degree Corner Exploration

Date: 2026-07-31 (Asia/Taipei)

## Decision

The exploratory corner test completed without tracking loss, but failed every
geometry gate. More time and a new viewing direction did not improve the map.
Map quality peaked during the first straight leg, then degraded throughout the
turn and second leg.

Do not proceed to a full square route.

## Test contract

Route:

```text
1.0 m straight
  -> in-place left turn 90 deg
  -> 0.5 m straight
  -> stationary for 10 s
```

Input and estimator:

- `640x360 @ 30 Hz`;
- 5 cm stereo baseline;
- 2.5 m simulation stereo render range;
- cuVSLAM 11.4 / Isaac ROS Visual SLAM release 2.1;
- IMU fusion disabled;
- speed `0.12 m/s`, turn rate `0.25 rad/s`;
- truth used only as a motion-profile stop condition and for post-run
  evaluation.

Bundle:

```text
/home/tseng/isaac_ros_ws/data/corner_exploration_1m_90deg_0p5m_20260731_59294
```

## Executed motion and tracking

- first-leg truth distance: `1.0104 m`;
- achieved truth turn: `91.89 deg`;
- second-leg truth distance: `0.5112 m`;
- minimum reported obstacle clearance: approximately `0.46 m`;
- all 883 status samples reported `vo_state=1`.

## Segmented trajectory result

| Phase | Truth | cuVSLAM | Error |
|---|---|---|---|
| First straight | 1.0104 m, 0.49° | 0.9048 m, 4.75° | 0.1618 m, 4.35° |
| Turn | 0 m, 88.75° | 0.4998 m, 64.64° | 0.4998 m, 24.23° |
| Second straight | 0.5112 m, 2.65° | 0.1952 m, 5.92° | 0.4380 m, 4.55° |
| Total | 1.1172 m, 91.89° | 0.9704 m, 72.60° | 0.2093 m, 19.61° |

The turn generated approximately 0.50 m of false translation while
underestimating rotation by 24.23 degrees.

Full-trajectory metrics:

| Metric | Result |
|---|---:|
| Translation ATE RMSE / P95 | 0.2704 / 0.4736 m |
| Rotation ATE RMSE / P95 | 15.14 / 21.92 deg |
| 1 s translation RPE RMSE / P95 | 0.0762 / 0.1926 m |
| 1 s rotation RPE RMSE / P95 | 2.34 / 6.48 deg |

Maximum truth/estimate boundary synchronization skew was `15 ms`.

## Current-observation geometry

Eighty sampled synchronized clouds contained 8,115 points:

- 83.2% within 0.25 m of an SDF surface;
- median `0.0415 m`;
- P90 `0.5244 m`;
- P95 `0.8273 m`;
- maximum `12.3396 m`;
- 21 of 880 clouds were rejected by the 100 ms truth synchronization gate.

In-range observations remained relatively strong:

| Forward depth | Within 0.25 m |
|---|---:|
| 0–1.5 m | 87.6% |
| 1.5–2.5 m | 87.0% |
| 2.5–4.0 m | 11.6% |
| Beyond 4.0 m | 0.0% |

Stereo did not fail globally during the turn. Residual false correspondence
and rotational motion estimation remained the critical failures.

## LL_MAP progression

Representative snapshots:

| Route state | Points | Within 0.25 m | P90 |
|---|---:|---:|---:|
| Straight, 0.481 m | 378 | 78.0% | 0.7191 m |
| Straight, 0.775 m | 608 | **80.4%** | 0.6743 m |
| Turn start, 1.15° | 773 | 79.0% | 0.6211 m |
| Turn, 21.20° | 1,009 | 72.4% | 0.6412 m |
| Turn, 30.51° | 1,106 | 68.3% | 0.7014 m |
| Turn complete, 91.89° | 1,997 | 69.5% | 0.6776 m |
| Final second leg | 2,934 | **65.7%** | **0.7267 m** |

Final map:

- 2,934 landmarks;
- median surface distance `0.0618 m`;
- P90 / P95 `0.7267 / 0.8642 m`;
- 65.7% within 0.25 m;
- 1,005 landmarks (34.3%) over 0.25 m;
- maximum error `12.3023 m`.

The quality curve disproves the tested hypothesis for this configuration:
additional time and a 90-degree viewpoint change caused the raw `LL_MAP` to
accumulate bad structure faster than the backend removed it.

## Exploratory-gate verdict

| Gate | Limit | Result | Verdict |
|---|---:|---:|---|
| Tracking | no sustained loss | all 883 at state 1 | Pass |
| Turn angle error | at most 15° | 24.23° | Fail |
| Turn false translation | at most 0.20 m | 0.4998 m | Fail |
| Final LL_MAP within 0.25 m | at least 75% | 65.7% | Fail |
| LL_MAP P90 | must not worsen | 0.6211 → 0.7267 m | Fail |
| Catastrophic outliers | must not increase materially | 1,005 final outliers | Fail |

## Next technical gate

The next experiment should isolate rotation rather than extend the route:

1. verify per-frame angular image motion and stereo synchronization during a
   low-rate in-place turn;
2. compare the declared camera angular motion with cuVSLAM's incremental
   rotational estimate;
3. audit and then enable simulated IMU fusion only after the
   optical-camera-to-IMU axis and timestamp contract passes;
4. repeat an isolated turn before attempting another corner.

Filtering the displayed landmark cloud alone cannot correct the 0.50 m false
translation created during rotation.

