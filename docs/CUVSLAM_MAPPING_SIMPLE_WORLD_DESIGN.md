# cuVSLAM Simple Mapping World Design

Updated: 2026-07-27 (Asia/Taipei)

## Decision

- Change class: C2 simulation launch/interface change.
- Environment: Jetson simulation only, ROS 2 Foxy host plus ROS 2 Humble
  Isaac ROS container, ROS domain 43.
- Observable objective: start an independent simple Gazebo world while keeping
  the verified D435i and ground-truth interfaces available to cuVSLAM.
- Non-goal: this change does not add autonomous motion, Nav2, dense occupancy
  mapping, IMU fusion, dynamic obstacles, or real-robot deployment.

## Options

### A. Independent world selected

Add `cuvslam_mapping_simple.sdf` and select it through the existing
`gz_sim.launch.py world:=...` argument.

- Advantages: preserves the verified baseline world, isolates geometry changes,
  supports deterministic rollback, and gives later tests a stable starting map.
- Disadvantages: the embedded `slam_bot` model is duplicated until a separately
  reviewed model extraction is justified.
- Contract compatibility: D435i, LiDAR, command, clock, and ground-truth topic
  names remain unchanged.
- Failure mode: a malformed world fails at Gazebo startup without modifying the
  baseline file.
- Deployment: copy only to the simulation workspace
  `/home/tseng/jetson_slam_ws`.
- Maintenance: compare robot blocks when the baseline sensor model changes.
- Extension: future worlds may add route complexity while this world remains
  the level-zero baseline.
- Rollback: stop the test world and launch the unchanged
  `indoor_gz_sim.sdf`.

### B. Modify the baseline world rejected

Add mapping geometry directly to `indoor_gz_sim.sdf`.

- Advantages: one world and no temporary model duplication.
- Disadvantages: invalidates the current D435i baseline and makes regressions
  harder to attribute.
- Contract compatibility: topics can remain stable, but environmental evidence
  would no longer match earlier reports.
- Failure mode: a world error removes the known-good simulation entry point.
- Deployment and rollback: require restoring a protected baseline copy.
- Maintenance: fewer files, but higher evidence and regression cost.
- Extension: later difficulty levels would accumulate in one mutable file.

The user approved Option A on 2026-07-27.

## Level-Zero Layout

- Room: 6 m by 6 m, 1.2 m walls.
- Robot start: `(-1.8, -1.8, yaw 0)`.
- Clear nominal square route:
  `(-1.8,-1.8) -> (1.8,-1.8) -> (1.8,1.8) -> (-1.8,1.8)`.
- Static geometry: one central crate and one short column.
- Visual landmarks: three deliberately non-symmetric high-contrast wall
  patterns.
- Lighting: one fixed ceiling point light.

## Preserved Contracts

| Signal | Contract |
|---|---|
| Stereo left | `/d435i/infra1/image_rect_raw` and camera info |
| Stereo right | `/d435i/infra2/image_rect_raw` and camera info |
| IMU | `/d435i/imu`; published but not fused in this milestone |
| LiDAR | `/scan` |
| Simulation clock | `/clock` |
| Robot command | ROS `/cmd_vel` to Gazebo `/model/slam_bot/cmd_vel` |
| Truth reference | `/ground_truth/odom`; evaluation only |
| Visual localization | cuVSLAM owns `map -> odom` in the current experiment |

Gazebo truth is not a localization input and must not be presented as real
camera accuracy.

## Acceptance Gates

1. The SDF passes an XML parser.
2. `slam_gazebo` builds after the world is installed.
3. Gazebo starts the world through the existing `world` launch argument.
4. Five required D435i topics are present.
5. `/clock`, `/cmd_vel`, `/ground_truth/odom`, and `/scan` are present.
6. cuVSLAM reaches `vo_state=1` from the simulated stereo pair.
7. The baseline `indoor_gz_sim.sdf` remains unchanged and selectable.

## Verification Evidence

Verified on Jetson through `tseng@192.168.2.2` on 2026-07-27:

- Local `xmllint --noout` passed.
- Parsed structure: world `cuvslam_mapping_simple`, 11 models, two cameras,
  one IMU, and one GPU LiDAR.
- `colcon build --symlink-install --packages-select slam_gazebo` passed:
  one package in 1.03 seconds.
- Source and installed world SHA-256 matched:
  `e55b19a27030b3d58a3e1e0e6c6d5d22683edcac66a5078eec7cf3eb34d5ce1a`.
- Independent transient units
  `cuvslam-mapping-simple-gazebo.service` and
  `cuvslam-mapping-simple-bridge.service` were both active.
- ROS domain 43 exposed `/clock`, `/cmd_vel`, `/scan`,
  `/ground_truth/odom`, all five required D435i topics, and ROS diagnostics.
- `/ground_truth/odom` had one publisher.
- `/d435i/infra1/image_rect_raw` had one publisher and one subscriber.
- cuVSLAM 11.4 started against this world and reported `vo_state=1`;
  `/visual_slam/status` had one publisher.
- The test bridge stopped normally. The test Gazebo server remained in
  `deactivating/final-sigterm` and required a unit-scoped `SIGKILL`. No test
  Gazebo process remained afterward.
- The GUI-managed baseline world was restored after the test. Its Gazebo and
  bridge services returned active, all five D435i topics became ready, and
  cuVSLAM again reported `vo_state=1`.

The test world was stopped after verification. The GUI-managed baseline world
and cuVSLAM were left running.

Not run in this change:

- Robot motion or square-route execution.
- Ground-truth odometry sample and trajectory comparison while moving.
- ATE, RPE, closed-loop error, tracking-loss, or relocalization metrics.
- Gazebo GUI visual inspection.
- IMU fusion, dense mapping, occupancy mapping, or Nav2.
- Visual Gazebo GUI inspection of either world.

## Rollback

Stop cuVSLAM, bridge, and Gazebo test processes. Restart the existing GUI
simulation workflow, which selects `indoor_gz_sim.sdf` by default. Delete only
the deployed `cuvslam_mapping_simple.sdf` if permanent removal is approved.
