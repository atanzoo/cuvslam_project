# cuVSLAM Initial Dense-v3 Mapping Trial

Date: 2026-07-31  
Bundle:
`/home/tseng/isaac_ros_ws/data/initial_mapping_native_corner_dense_v3_compact_20260731_108473`

## Purpose

Run the first controlled mapping route after the in-place turn pose fix:

```text
1.0 m straight -> 90 deg left turn -> 0.5 m straight
```

The run used the dense-turn v3 world, stereo-only cuVSLAM, mapping enabled,
20 deg/s turn command, 0.12 m/s linear command, and camera x offset 0.19 m.
Wheel odometry was absent from the bridge, controller, bag, cuVSLAM, and
evaluation.

Linear stopping used a compact Gazebo native-pose relay. The relay extracts
only the `slam_bot` pose on the Jetson host so the large 131-transform
`Pose_V` message is not sent into the cuVSLAM container. Turn stopping used
timestamped D435i gyro integration. Native pose was never an estimator input.

## Route Execution

- first leg controller: 1.0023 m;
- turn controller: 90.04 deg IMU integration;
- second leg controller: 0.5007 m;
- cuVSLAM status: 852 / 852 samples at `vo_state=1`;
- wheel odometry topic in bag: absent.

Sensor timing remained valid:

- stereo: 28.57 Hz measured, 35 ms median period;
- IMU: 200.00 Hz, 5 ms median period, 20 ms maximum;
- native turn: 90.48 deg;
- integrated gyro over native turn: 90.50 deg;
- native/IMU difference: 0.02 deg.

## Trajectory Result

| Phase | Native translation | cuVSLAM translation | Translation residual | Yaw error |
|---|---:|---:|---:|---:|
| First leg | 1.0110 m | 0.9151 m | 0.1010 m | -3.14 deg |
| Turn | 0.0230 m | 0.0508 m | 0.0461 m | -11.59 deg |
| Second leg | 0.5094 m | 0.5001 m | 0.0118 m | -0.14 deg |
| Complete route | 1.1126 m | 1.1347 m | 0.0509 m | -14.88 deg |

Trajectory translation error was 0.0510 m median, 0.1231 m P95, and
0.1506 m maximum. Rotation error was 16.22 deg median and 16.42 deg P95.
The route pose gate failed.

The pure-turn dense v3 gate passed when rotation occurred at the initial
position `(-1.8, -1.8)`. This route rotates after moving approximately one
metre, near `(-0.8, -1.8)`, outside the dense feature arc's intended center.
The renewed 11.59-degree yaw under-estimation therefore supports a
position-dependent scene-observability cause rather than an IMU or controller
regression.

## Current-Observation Geometry

Forty uniformly sampled current-observation clouds contained 4,348 points:

- 98.1 percent within 0.10 m of an SDF surface;
- 99.3 percent within 0.25 m;
- surface-distance median 0.0041 m;
- P90 0.0339 m;
- P95 0.0498 m;
- 0.7 percent over 0.25 m.

Current stereo triangulation remained highly accurate. The failed route pose
is therefore upstream of map accumulation but is not explained by a general
observation-geometry collapse.

## Final Landmark Geometry

The final map contained 4,243 landmarks:

- 86.6 percent within 0.10 m of an SDF surface;
- 99.5 percent within 0.25 m;
- median 0.0270 m;
- P90 0.1188 m;
- P95 0.1457 m;
- 22 points (0.5 percent) over 0.25 m;
- maximum outlier distance 54.4025 m.

The map metric improved sharply from the earlier approximately 66-percent
result, but it must not be interpreted as a passed global map. Most landmarks
lie on the dense feature arc and broad nearest-surface scoring can hide the
wrong post-turn orientation. Snapshot P90 degraded from approximately
0.043 m before the turn to 0.1188 m at route completion, and the 54.4 m
extreme outliers remain unacceptable.

## Decision

The initial mapping pipeline successfully produced a map and preserved
tracking, sensor timing, controller truth, and wheel-odometry isolation.
However, the map is not accepted because route pose failed:

- first straight under-estimation is at the 10 cm boundary;
- turn yaw is under-estimated by 11.59 deg;
- final route yaw is wrong by 14.88 deg;
- extreme map outliers remain.

## Next Gate

Extend continuous asymmetric near-field geometry around the actual corner
location `(-0.8, -1.8)`, not only around the initial pose. Repeat the exact
same route and require:

- first- and second-leg translation residual no greater than 0.10 m;
- turn yaw error no greater than 5 deg;
- turn false translation no greater than 0.05 m;
- final route translation residual no greater than 0.15 m;
- zero tracking loss;
- final landmarks at least 95 percent within 0.25 m;
- no catastrophic map outlier beyond the declared render/evaluation range.
