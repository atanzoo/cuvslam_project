# cuVSLAM Stage 3 Image Timing And Motion Sampling

Date: 2026-07-30  
Environment: Jetson AGX Orin simulation, ROS domain 43  
Result: 30 Hz real-time budget failed; 20 Hz timing passed but trajectory regressed  
Final runtime state: Simulation stopped

## Scope

This stage retained the validated intrinsics, baseline, TF, left/right topic
ordering, rectification, image dimensions, world geometry, LiDAR, and cuVSLAM
parameters.

It measured:

- image stamp and wall-arrival cadence;
- exact stereo pairing and interior unmatched frames;
- Gazebo real-time factor;
- image mean, contrast, clipping, and temporal change;
- static and motion optical flow;
- cuVSLAM callback and tracking execution time.

Motion used the established safety profile. It subscribed only to `/scan` and
`/visual_slam/status`; ground truth remained offline evaluation data.

## 30 Hz Baseline

All image streams maintained approximately `30.3 Hz` in simulation time.
Maximum stamp gaps were `35 ms`, exposure and contrast passed, static optical
flow was zero, and every status sample remained `vo_state=1`.

| Phase | RTF | Left arrival P95 | Track mean | Track P95 | Track max | Flow P95 |
|---|---:|---:|---:|---:|---:|---:|
| Static | 0.716 | 63.48 ms | 4.06 ms | 8.43 ms | 22.02 ms | 0.00 px |
| Straight | 0.760 | 61.08 ms | 5.00 ms | 10.76 ms | 59.68 ms | 3.68 px |
| Turn | 0.742 | 61.90 ms | 5.20 ms | 11.32 ms | 76.58 ms | 1.61 px |

The exact-pair fractions were `99.45-99.51%`; each observation window differed
by only one subscription-edge frame. The original monitor incorrectly treated
that edge frame as a 30-35 ms stereo skew. The corrected metric distinguishes
edge loss from interior loss. Follow-up runs measured `0 ms` true skew and no
interior unmatched frames.

The baseline fails because RTF is far below `0.95`, and motion contains
occasional tracking calls longer than the 30 Hz `33.3 ms` frame budget.
Typical processing is not overloaded: P95 remains around `11 ms`.

## Load Isolation

With cuVSLAM stopped but the same 30 Hz Gazebo, bridge, and image monitor:

```text
RTF=0.808
left/right stamp rate=30.32/30.31 Hz
true stereo skew=0 ms
interior unmatched frames=0
```

cuVSLAM contributes additional load, but Gazebo's dual-camera rendering and
transport already prevent real-time execution on this configuration.

## Controlled 20 Hz Variant

A parser-generated temporary SDF changed only the two infrared camera
`update_rate` values from 30 to 20 Hz. It did not overwrite the original world.

All timing phases passed:

| Phase | RTF | Pair fraction | True skew | Track max | Flow P95 |
|---|---:|---:|---:|---:|---:|
| Static | 0.962 | 99.43% | 0 ms | 12.24 ms | 0.00 px |
| Straight | 0.964 | 100% | 0 ms | 36.17 ms | 5.03 px |
| Turn | 0.960 | 99.48% | 0 ms | 23.51 ms | 2.61 px |

The 20 Hz frame budget is `50 ms`, so every tracking call passed.

## Trajectory Result

Timing success did not translate into valid odometry.

### Static

```text
truth drift=0
cuVSLAM drift=0
endpoint skew=0 ms / 0 ms
```

### Straight

```text
ground truth=0.4776 m
cuVSLAM distance=0.6704 m
distance relative error=40.4%
false rotation=4.46 deg
base-motion residual=1.0290 m
```

The estimated displacement direction was also inconsistent with ground truth,
so the distance percentage alone understates the failure.

### Turn

```text
ground-truth rotation=33.14 deg
cuVSLAM rotation=32.75 deg
angle magnitude error=1.2%
false base translation=3.0365 m
base residual rotation=7.00 deg
```

The false turn translation regressed from the prior 30 Hz high-contrast trial's
`0.3865 m` to `3.0365 m`. The 20 Hz variant is therefore rejected despite
passing every timing gate.

## Stage Decision

The simulation has a real wall-time performance limitation at 30 Hz:

- Gazebo RTF is only about `0.72-0.76` with cuVSLAM;
- occasional tracking calls exceed the 30 Hz frame budget;
- reducing the rate to 20 Hz restores near-real-time execution.

However, timing pressure is not sufficient to explain or fix the pose defect.
The timing-corrected 20 Hz run still failed and produced a much larger false
turn translation. The original 30 Hz camera contract remains unchanged.

Proceed to Stage 4: verify cuVSLAM's exact assumptions for simulated rectified
stereo input, including frame/extrinsic interpretation, calibration ownership,
image format, and supported simulated-camera semantics.

## Evidence

- `tools/measure_sim_image_timing.py`
- `tools/test_measure_sim_image_timing.py`
- `tools/create_sim_camera_rate_variant.py`
- `tools/test_create_sim_camera_rate_variant.py`
- `docs/CUVSLAM_STAGE3_IMAGE_TIMING_DESIGN.md`
- `reports/stage3_static.json`
- `reports/stage3_straight.json`
- `reports/stage3_turn.json`
- `reports/stage3_static_without_cuvslam.json`
- `reports/stage3_static_20hz.json`
- `reports/stage3_straight_20hz.json`
- `reports/stage3_turn_20hz.json`
- Jetson experiment:
  `/workspaces/isaac_ros-dev/data/experiments/stage3_image_timing_20260730`

Local tests:

```text
Ran 8 tests
OK
```

## Final Shutdown Evidence

```text
cuvslam-stage3-20hz-gazebo.service: inactive
cuvslam-stage3-20hz-bridge.service: inactive
cuvslam-stage3-gazebo.service: inactive
cuvslam-stage3-bridge.service: inactive
cuVSLAM PID count: 0
recorder/profile/monitor PID count: 0
/clock: no message during a 3-second sample
```
