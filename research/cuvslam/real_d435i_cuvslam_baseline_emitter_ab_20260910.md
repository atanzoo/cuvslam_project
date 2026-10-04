# Real D435i cuVSLAM-only baseline and emitter A/B result

Date: 2026-09-10 (Asia/Taipei)

## Decision

For the currently tested indoor space, the operational odometry baseline is:

```text
D435i + IMU -> cuVSLAM -> odom -> base_link
```

The RPLIDAR A2M12 mapping path remains available for map observation, but LiDAR
scan-matching odometry and EKF fusion are deferred as a future enhancement.
Their code, configuration, shadow topics, tests, and rollback evidence are
retained; they are not promoted to the formal `odom -> base_link` owner.

The current emitter A/B decision for this space is:

```text
Emitter OFF -> current preferred cuVSLAM baseline
Emitter ON  -> retained as the A/B comparison setting
```

This decision is scoped to the current D435i installation, room, lighting,
motion route, and 640x360x30 profile. It is not a universal RealSense rule.

## Environment and runtime

- Hardware: Jetson with Intel RealSense D435i; RPLIDAR A2M12 remained present
  for the mapping display but was not used as an odometry estimator.
- Runtime: `mode=imu`, `mapping=mapping`, `profile=640x360x30`,
  `lighting=official`, `diagnostics=standard`.
- Estimator: `cuvslam` only; `lidar_odom=false`, `ekf_shadow=false`.
- Requested D435i settings: emitter OFF, automatic exposure ON, requested
  exposure 8500 us, gain 16, denoise ON.
- TF ownership: cuVSLAM remains the only local odometry owner;
  `slam_toolbox` remains the mapping-side `map -> odom` owner.

## A/B evidence

The GUI A/B selector forwarded the requested emitter state to the Jetson and
the work log, trace, and session preserved the selected value.

Primary OFF run:

- Trace: `real_robot/cuvslam/evidence/logs/real_d435i_quality/20260910_163200_r2_trace.jsonl`
- Actual metadata repeatedly reported `Emitter L/R: OFF / OFF`.
- Final odometry telemetry: approximately 30 Hz, VO valid ratio 100%,
  `x=0.067 m`, `y=0.014 m`, `yaw=-2.24°`, path length `14.98 m`.
- The displayed path was visually continuous and returned close to its start.
- Automatic exposure varied substantially, reaching approximately 20.2 ms at
  one point; this remains a low-light/motion risk even though the full route
  result was good.

ON comparison traces from the same A/B session are retained:

- `real_robot/cuvslam/evidence/logs/real_d435i_quality/20260910_163132_r2_trace.jsonl`
- `real_robot/cuvslam/evidence/logs/real_d435i_quality/20260910_163458_r2_trace.jsonl`

The operator's controlled A/B result for the tested route was that OFF gave the
more stable and visually usable cuVSLAM trajectory. The individual route and
turn metrics remain the acceptance evidence; a visually smooth path alone is
not treated as universal accuracy proof.

## Interpretation and limitation

The result supports using emitter OFF as the current local-odometry baseline in
this room. A plausible mechanism is that the D435i projector pattern creates
tracking disturbance for this particular scene and camera placement. However,
the OFF trace also recorded intermediate rotation geometry warnings, including
turn candidates with approximately `0.595--1.555 m` maximum XY displacement.
Therefore the correct claim is:

> Emitter OFF produced the best observed cuVSLAM-only result for the current
> space and test route; it does not prove that every local turn transient is
> solved or that emitter OFF is best in every environment.

## Navigation boundary

For the next navigation work, use the cuVSLAM-only baseline and evaluate the
complete local odometry contract before adding fusion:

1. stationary startup and TF check;
2. labelled straight motion;
3. labelled turns and closed loop;
4. Nav2 odometry/costmap integration check;
5. tracking-loss and recovery behavior.

LiDAR odometry + EKF should return only when cuVSLAM-only navigation exposes a
repeatable failure that the additional source is expected to address, and when
the RF2O covariance, frame, timestamp, direction, and Jetson resource gates
are all qualified.

## Rollback

No runtime code was removed. To compare again, select emitter ON in the GUI or
select the retained `ekf_shadow` estimator for a diagnostic-only comparison.
The formal TF ownership and the existing cuVSLAM-only path remain unchanged.
