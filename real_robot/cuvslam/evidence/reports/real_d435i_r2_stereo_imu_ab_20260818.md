# Real D435i R2 stereo / stereo+IMU A/B — 2026-08-18

## Shared setup

- D435i serial `261222077990`, firmware `5.15.1.55`, USB 3.2
- Stereo `848x480` mono8 at nominal 30 Hz
- cuVSLAM 11.4, Isaac ROS Visual SLAM 2.1.0
- `enable_mapping=true`, `initial_reset=false`
- output frame contract: `odom -> camera_link`
- both runs recorded stereo, CameraInfo, IMU, TF, status, odometry, VO pose,
  SLAM path, and landmarks topics

## Bag-level comparison

| Metric | Stereo-only | Stereo + IMU |
|---|---:|---:|
| Fusion setting | `false` | `true` |
| Bag duration | 30.314 s | 32.191 s |
| Left/right images | 910 / 910 | 967 / 967 |
| IMU messages | 6,049 | 6,422 |
| Odometry messages | 807 | 800 |
| Status messages | 807 | 800 |
| `vo_state=1` | 807/807 | 800/800 |
| End displacement | 0.7028 m | 0.3945 m |
| Path length | 1.1640 m | 0.4542 m |
| Start-to-end rotation | 14.37° | 3.91° |

## Assessment

Both R2 modes launched on the real D435i and produced complete stereo, IMU,
odometry, status, TF, and SLAM-path recordings. Tracking remained active for
100% of the recorded status messages in both modes. The stereo+IMU launch also
logged `Enable IMU Fusion: true` and used the R1.5 IMU noise parameters.

These two routes were operator-driven and not geometrically identical, so the
numbers are an operational A/B indication, not a final accuracy claim. The
IMU-fusion route had less final rotation in this pair, but a controlled repeat
with the same start point, distance, speed, and heading is still required for
the thesis comparison.

The RealSense driver emitted intermittent Motion Module/USB watchdog warnings
during startup, although the current IMU stream measured about 199.5 Hz and
the bag contained the expected data. Keep this warning in the experiment log
until a power-cycle repeat shows it is consistently absent.

The large rosbag files remain on Jetson under
`/home/tseng/isaac_ros_data/experiments/real_d435i_r2/`; local JSON/metadata
copies are under `logs/real_d435i_r2/`.
