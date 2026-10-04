# 2026-09-24 current cuVSLAM + A2M12 map

## Artifact

- Source topic: `/map`
- Source runtime: Jetson container `real_d435i_rplidar`
- Saved with: `nav2_map_server/map_saver_cli`
- Save tag: `20260924_092630`
- Map size: `199 x 308` cells
- Resolution: `0.05 m/cell`
- YAML origin: `[-3.89, -5.34, 0]`

Remote source directory:

`/home/tseng/isaac_ros_data/real_robot/cuvslam/maps/20260924_092630_current_cuvslam_a2m12/`

Local downloaded directory:

`real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/`

## Integrity

| File | SHA-256 |
|---|---|
| `map.pgm` | `41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74` |
| `map.yaml` | `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d` |

The local hashes match the Jetson files. This step only saved and downloaded
the map; localization, Nav2, `/cmd_vel`, and robot motion were not started.
