# Jetson workspace organization record — 2026-08-31

Status: filesystem migration complete; runtime and hardware acceptance remain
pending.

## Objective

Organize the Jetson Isaac ROS runtime by workstream and preserve a traceable
record for future fixes, without copying the large workspace or changing
cuVSLAM, TF, topic, QoS, safety, or Docker-image behavior.

## Environment

- Target: Jetson AGX Orin Developer Kit.
- Workspace: `/home/tseng/isaac_ros_ws`.
- Simulation data root: `/home/tseng/isaac_ros_ws/simulation/cuvslam/data`.
- Real D435i data root: `/home/tseng/isaac_ros_data/real_robot/cuvslam`.
- ROS/Docker runtime: Isaac ROS Humble container on host ROS 2 Foxy.
- Active display container during migration: `real_d435i_rviz`.
- Active display image: `isaac_ros_dev-aarch64:rviz-vnc`.

## Before state and storage

The Jetson workspace had a mixed top-level `tools/` directory and a separate
real-data root. The verified pre-migration sizes were:

```text
/home/tseng/isaac_ros_ws/data       3.0G after the approved cleanup
/home/tseng/isaac_ros_data          4.7G
/home/tseng/isaac_ros_ws/src        658M
/home/tseng/isaac_ros_ws/build      114M
/home/tseng/isaac_ros_ws/install    16M
/home/tseng/isaac_ros_ws/log        20M
```

The Docker images occupy about 16.9G in shared layers. The active RViz
container was not stopped and no Docker image was removed.

## Approved cleanup

The following five unreferenced GUI intermediate runs were deleted after an
existence and process check:

```text
/home/tseng/isaac_ros_ws/data/gui_native_corner_20260803_142814
/home/tseng/isaac_ros_ws/data/gui_native_corner_20260803_145636
/home/tseng/isaac_ros_ws/data/gui_native_corner_20260803_150327
/home/tseng/isaac_ros_ws/data/gui_native_corner_20260803_151046
/home/tseng/isaac_ros_ws/data/gui_native_corner_20260803_151221
```

This released approximately 770M. The deletion is not recoverable from the
Jetson. Report-linked experiments, baseline/final runs, maps, calibration, and
other acceptance evidence were not deleted.

## Filesystem migration

The migration used same-filesystem moves, not copies:

```text
/home/tseng/isaac_ros_ws/real_robot/cuvslam/
  launch/  config/  tools/  calibration/
/home/tseng/isaac_ros_ws/simulation/cuvslam/
  tools/  config/  data/
/home/tseng/isaac_ros_data/real_robot/cuvslam/
  experiments/  bags/  maps/  logs/  calibration/  runs/
```

Real D435i tools and the real launch file were moved under
`real_robot/cuvslam/`. Simulation analysis, replay, Gazebo, and acceptance
tools were moved under `simulation/cuvslam/`. The existing workspace source,
build, install, and log trees were not duplicated.

Compatibility paths remain as symlinks so the current container and older
entry points continue to resolve:

```text
/home/tseng/isaac_ros_ws/tools/*
/home/tseng/isaac_ros_ws/config
/home/tseng/isaac_ros_ws/data
/home/tseng/isaac_ros_data/experiments
```

The active R1.5/R2 scripts now use
`/home/tseng/isaac_ros_data/real_robot/cuvslam` as their data root. The Mac
web GUI download paths were updated to the same canonical root.

The external simulation deployment helper was also aligned with the
simulation data root. Backups were created before editing:

```text
/home/tseng/isaac_ros_deployment/run.sh.bak_20260831_workspace_org
/home/tseng/isaac_ros_deployment/isaac_ros.env.example.bak_20260831_workspace_org
/home/tseng/isaac_ros_deployment/README.md.bak_20260831_workspace_org
/home/tseng/isaac_ros.env.bak_20260831_workspace_org
```

## Evidence and rollback

The migration manifests are retained on the Jetson:

```text
/home/tseng/isaac_ros_ws/manifests/jetson_workspace_20260831_before.tsv
/home/tseng/isaac_ros_ws/manifests/jetson_workspace_20260831_moves.tsv
/home/tseng/isaac_ros_ws/manifests/jetson_workspace_20260831_after_tree.txt
```

Recorded checksums:

```text
before manifest: d6211f32086882158eedd170dcce3a25fbb766e863e9267f3be64383c3051da8
move log:        584ee12714c35d94ef20b197c96726d33d8784e495588acddba7dadae9f7d507
after tree:      689497da14ac29ec509c8ae63ce8d24a3b5f9f57d97aa53f006cfd215f37f5c0
after manifest:   f22169c4a2abb44ac219572957d797a75224bbd55e0bd6fdf590b8f5a5a169b7
```

To roll back the migration, stop any runtime that uses the affected paths,
remove only the compatibility symlinks, move the canonical files back using
`jetson_workspace_20260831_moves.tsv`, restore the original data root, and
revert the two active script `DATA_DIR` values plus the Mac GUI constants. The
five approved GUI intermediate runs cannot be restored by this rollback.

## Verification

Passed:

- all canonical real/simulation files are present;
- all top-level compatibility symlinks resolve;
- all canonical real and simulation shell scripts pass `bash -n`;
- R1.5 and R2 `status` commands resolve from canonical paths;
- the active `real_d435i_rviz` container can read the RViz config and bridge
  source through its existing read-only workspace mount;
- disk usage remained at about 8.8G free after migration.

Not run:

- `colcon build`;
- D435i/R2 runtime launch;
- RPLIDAR driver or `slam_toolbox` launch;
- TF integration or mapping accuracy test;
- robot motion or safety test.

The current arrangement is a storage-constrained in-place separation. It does
not yet establish a fully independent real-robot workspace; that remains a
future deployment decision requiring additional storage and review.

## Next step

Use the canonical real paths for the next D435i/RPLIDAR preparation. Add the
RPLIDAR driver, `base_link -> rplidar_link` static TF, and mapping launch only
as a separate reviewed change. The intended ownership remains:

```text
cuVSLAM:     odom -> base_link
RPLIDAR SLAM: map -> odom
static TF:   base_link -> rplidar_link
```
