# GUI native corner mapping result

Date: 2026-08-03 11:04--11:06 (Asia/Taipei)  
World: `cuvslam_mapping_observable_640x360_range2p5_dense_turn.sdf`  
cuVSLAM: 11.4, stereo-only, mapping enabled, IMU fusion disabled  
Motion: native-pose controlled `1 m -> 90 deg -> 0.5 m`, turn command `20 deg/s`

## Execution

The GUI completed all stages:

- D435i topics: 5/5 ready.
- cuVSLAM: `vo_state=1` throughout the recorded status samples (`813/813`).
- Foxglove tunnel and ground-truth outline: ready.
- Native route: first leg `1.0005 m`, IMU turn `90.11 deg`, second leg
  `0.5019 m`.
- Bag size: `428 MiB`; mapping output was preserved.

The evaluator uses rosbag2 recorder timestamps for cross-topic matching. This
bag contains different message-header epochs for the Gazebo native relay and
cuVSLAM odometry; using their headers directly would incorrectly report zero
time overlap.

## Pose result

| Phase | Native | cuVSLAM | Error |
|---|---:|---:|---:|
| First leg translation | 1.0080 m | 0.6128 m | 0.4027 m |
| Turn yaw | 88.26 deg | 13.04 deg | -75.22 deg |
| Turn translation | 0.0230 m | 0.3248 m | 0.3380 m |
| Second-leg yaw | 2.52 deg | 65.39 deg | +62.87 deg |
| Complete-route yaw | 91.57 deg | 75.00 deg | -16.57 deg |
| Complete-route translation | 1.1103 m | 1.1753 m | 0.0713 m |

Trajectory error was `0.1689 m` median, `0.4271 m` P95, and `15.20 deg`
median rotation error (`63.97 deg` P95). Tracking stayed active, but the pose
gate failed because the turn orientation and false translation are too large.

## Elevation / attitude

Gazebo native pose remained level: maximum absolute roll and pitch were both
`0.000 deg`. cuVSLAM was substantially better than the previous `5 deg/s`
run, but it was not level enough to accept:

- cuVSLAM maximum absolute roll: `3.342 deg`.
- cuVSLAM maximum absolute pitch: `5.402 deg`.
- Final cuVSLAM RPY: `(+2.886, +1.519, +75.008) deg`.

Therefore the SDF chassis and ground-truth outline are not the source of the
elevation mismatch. The `20 deg/s` route reduced the vertical attitude drift,
but this full corner route still exposes a cuVSLAM rotation/translation failure.

## Landmark geometry

The final landmark cloud contained `4,335` points. Against the static SDF
surfaces:

- median surface distance: `0.0368 m`;
- P90: `0.1393 m`;
- P95: `0.1880 m`;
- within `0.25 m`: `4,272/4,335` (`98.5%`);
- maximum outlier: `41.2481 m` (`63` points outside `0.25 m`).

This is a geometrically strong final cloud compared with the earlier roughly
66% result, but it cannot override the failed trajectory gate. The endpoint
translation happens to be close while the turn orientation is wrong, so this
map should be treated as a diagnostic map rather than an accepted navigation
map.

## Conclusion

The minimal correction was correct and useful: the GUI now uses the validated
dense-v3 `20 deg/s` turn profile, and the apparent elevation mismatch dropped
from about `18.8 deg` in the old `5 deg/s` run to at most `5.4 deg`. However,
the complete route still fails pose acceptance because cuVSLAM estimated only
about `13 deg` during the measured `88 deg` turn and compensated with a large
false translation. The next experiment should isolate the full-route timing
and scene observability around the turn; do not enable wheel odometry or use
the SDF/native pose to correct cuVSLAM.

## Cleanup policy implemented

GUI start/stop/close now stops route, cuVSLAM launch children, Gazebo, bridge,
and stale `ign-transport-topic --json-output` writers. It removes only
regenerable old ROS logs and transient runtime logs, with a disk preflight
before start. Rosbags, SDFs, reports, and mapping output are preserved.
