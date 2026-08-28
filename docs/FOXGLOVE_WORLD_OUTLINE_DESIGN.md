# Foxglove Ground-Truth World Outline Design

Date: 2026-07-28 (Asia/Taipei)
Status: Selected for staged simulation deployment
Change class: C5 package boundary plus C2 ROS interfaces

## Objective

Overlay the known Gazebo Baseline world geometry in Foxglove so an operator can
compare cuVSLAM paths, landmarks, and LiDAR measurements against simulation
ground truth.

This is evaluation-only observability. Ground truth must not feed cuVSLAM,
navigation, control, or a localization transform used by the robot.

## Interfaces

- Input: Baseline `indoor_gz_sim.sdf`
- Input: `/ground_truth/odom` (`nav_msgs/msg/Odometry`)
- Input TF: `map -> camera_infra1_optical_frame`
- Output TF: `map -> simulation_world_ground_truth`
- Output: `/simulation/ground_truth_world_markers`
  (`visualization_msgs/msg/MarkerArray`)

The output child frame and topic contain `ground_truth` explicitly. No existing
TF child, localization topic, command topic, or simulator service is replaced.

## Option A: Dedicated Observability Package (Selected)

Add `cuvslam_sim_observability` as an independent ROS 2 Python package in the
Isaac ROS simulation workspace.

Advantages:

- One modular responsibility: evaluation visualization.
- Clear simulation-only dependency and lifecycle.
- Parses the declared SDF instead of duplicating marker coordinates in code.
- Can later publish reference trajectories, alignment residuals, and map error
  metrics without changing cuVSLAM.
- Can be tested independently from Foxglove.

Disadvantages:

- Adds one package and build/deployment step.
- Keeps a packaged snapshot of the Baseline SDF; checksum synchronization is
  required.
- Initial alignment uses simulation ground truth and must remain isolated.

Failure modes:

- Missing SDF: node exits without affecting simulation or cuVSLAM.
- Missing ground-truth odometry or map TF: no ground-truth frame or markers are
  published.
- Stale SDF snapshot: outlines differ from Gazebo; checksum check must fail
  deployment verification.
- Restart during a run: the evaluation frame realigns at restart and cannot be
  used to measure drift accumulated before that restart.

Deployment impact:

- Build one `ament_python` package in the Humble Isaac ROS workspace.
- Launch the node with the simulation cuVSLAM launch.
- Add `/simulation/*` to the read-only Foxglove topic whitelist.

Maintenance:

- Package owner: simulation operations and observability.
- The Baseline SDF checksum is recorded in the deployment report.

Rollback:

- Restore the backed-up cuVSLAM launch and Foxglove launcher.
- Stop the cuVSLAM launch and remove or ignore the observability package.
- No Gazebo world, real-robot workspace, Nav2, or localization provider changes
  are required.

## Option B: Hard-Coded Markers In The cuVSLAM Launch (Rejected)

Place wall and box coordinates directly in the existing cuVSLAM launch or GUI.

Advantages:

- Fewer files and no package build.
- Quick for one screenshot.

Disadvantages:

- Duplicates the world geometry and silently drifts from the SDF.
- Violates the cuVSLAM package responsibility.
- Has no reusable alignment or test boundary.
- Future worlds require source edits in an unrelated module.

This option was rejected because it is cheaper only for the first run and
creates an unreliable comparison contract.

## Alignment

At the first synchronized ground-truth sample and available cuVSLAM TF:

```text
T_map_odom = T_map_camera * inverse(T_odom_base * T_base_camera)
T_map_world = T_map_odom * inverse(T_world_base_initial)
```

`T_base_camera` and `T_world_base_initial` come from the Baseline SDF. The node
then publishes the latched evaluation transform:

```text
map -> simulation_world_ground_truth
```

All world markers stay in `simulation_world_ground_truth`. This aligns the
known world once at experiment start while leaving later cuVSLAM drift visible.

## Verification

1. Package builds in the Humble container.
2. Packaged SDF checksum equals the active Gazebo Baseline SDF.
3. Marker topic uses `MarkerArray` and contains floor, four walls, and both
   feature boxes.
4. `map -> simulation_world_ground_truth` has exactly one publisher.
5. Foxglove bridge advertises and subscribes to the marker topic.
6. Foxglove visually overlays wall markers with `/scan`.
7. cuVSLAM remains `vo_state=1`.
8. Full GUI stop/start remains repeatable.

Official ROS Humble defines `visualization_msgs/msg/MarkerArray` for arrays of
non-interactive 3D visualization markers:

```text
https://docs.ros.org/en/humble/p/visualization_msgs/
```
