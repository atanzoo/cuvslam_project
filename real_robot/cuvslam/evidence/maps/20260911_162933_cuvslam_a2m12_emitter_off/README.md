# 2026-09-11 cuVSLAM + A2M12 map candidate

## Artifact

- Local directory: `real_robot/cuvslam/evidence/maps/20260911_162933_cuvslam_a2m12_emitter_off/`
- Source topic: `/map` (`slam_toolbox`, `nav_msgs/msg/OccupancyGrid`)
- Jetson source runtime: `real_d435i_rplidar`
- Saved with `nav2_map_server map_saver_cli`
- Capture/save tag: `20260911_162933`
- Map size: `200 x 296` cells
- Resolution: `0.05 m/cell`
- YAML origin: `[-6.29, -9.62, 0]`

The map was downloaded from Jetson after the mapping run. The corresponding
remote directory is:

`/home/tseng/isaac_ros_data/real_robot/cuvslam/maps/20260911_162933_cuvslam_a2m12_emitter_off/`

## Runtime context

- Estimator: cuVSLAM-only odometry; LiDAR odometry and EKF shadow were not used
  for this map.
- D435i IR emitter: OFF.
- Temporary software extrinsic used for the fixed A2M12 mount:
  `base_link -> rplidar_link`, `x=0.0`, `y=-0.10`, `z=0.0`, `yaw=2.334 rad`.
- The map is a candidate for the next localization experiment. It is not yet
  a localization, navigation, or `/cmd_vel` acceptance result.

The live GUI showed scan-map endpoint alignment of approximately `1% within
0.30 m` at capture. Therefore this artifact is preserved for controlled
localization testing, but that alignment value must not be treated as a formal
map-accuracy pass.

## Integrity

| File | SHA-256 |
|---|---|
| `map.pgm` | `a1386fa936614f92c6735250daad3702e64df037edea4a15db73b38c5176b63e` |
| `map.yaml` | `d3fcb45a4a3747f3f2b48924167234d11a0c9e5369f9e16511000fa8c1835971` |

The local hashes match the files saved on Jetson. No localization or robot
motion was started as part of this download step.
