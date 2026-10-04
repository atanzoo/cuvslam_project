# Mac Foxglove Simulation Viewer Test Report

Date: 2026-07-28 (Asia/Taipei)

## Scope And Classification

- Environment: Gazebo simulation on Jetson AGX Orin; no real robot.
- Change class: C5 simulation observability package plus C2 operator interface.
- ROS: Foxy host, Humble Isaac ROS container, ROS domain 43.
- Mac viewer: Foxglove desktop 2.57.0.
- Container bridge: `foxglove_bridge 0.7.2`.
- A simulation-only ROS package and cuVSLAM simulation launch were modified.
  No active Gazebo world, real-robot workspace, production Nav2 file, or motor
  interface was modified.

## Design Decision

Selected option: run the installed Humble Foxglove bridge inside
`isaac_ros_dev-aarch64-container`, bind it to Jetson localhost, and forward it
to Mac localhost through SSH.

Advantages:

- Reuses an installed package and the container that can see domain 43.
- Does not expose the bridge on Jetson network interfaces.
- Keeps the Mac viewer outside ROS DDS discovery and distribution concerns.
- Has one explicit launcher-owned lifecycle and rollback path.

Failure modes:

- Closing the launcher Terminal disconnects Foxglove.
- Jetson localhost port 8765 or Mac localhost port 8766 already in use prevents
  startup.
- SSH loss ends the tunnel.
- The old bridge version may lack newer Foxglove capabilities.

Rejected option: build Foxglove Bridge from source on the Foxy host.

Reason: Foxglove documents a source build for ROS distributions older than
Humble. It adds a second package and maintenance path without improving this
viewer workflow.

### Consolidated Start Workflow

Selected option: keep the existing component owners and orchestrate them from
the Mac GUI in this order: D435i/Gazebo, cuVSLAM readiness, Foxglove, then the
low-rate monitor.

- Compatibility: existing services, topics, and launch files remain the
  component entry points.
- Failure behavior: the log names the failed stage; completed earlier stages
  remain active for diagnosis.
- Deployment: one Mac GUI change plus one simulation-only TF addition.
- Maintenance: startup policy remains in the operator layer.
- Rollback: restore the previous GUI and cuVSLAM launch file.

Rejected option: add a new Jetson systemd meta-service that owns Gazebo,
cuVSLAM, and Foxglove together.

- It would hide component state behind another service and couple a Mac-only
  viewer to Jetson runtime ownership.
- Partial failures and independent component restarts would be harder to
  diagnose.
- It adds deployment and lifecycle maintenance without changing the ROS
  contract.

### LiDAR Visualization Transform

Selected option: publish the simulation-only static transform from the
cuVSLAM launch. The transform is derived from the Baseline SDF poses:
camera `(0.19, 0.025, 0.20)` and LiDAR `(0, 0, 0.13)`, including the ROS
optical-frame rotation.

Rejected option: introduce `robot_state_publisher` and a new URDF solely for
Foxglove. That is the preferred future model-owner design when the full robot
description is ready, but it would add a new model contract and package scope
for one missing simulated sensor transform.

### Ground-Truth World Outline

Selected option: add the independently owned simulation package
`cuvslam_sim_observability`. It parses the packaged Baseline SDF, publishes
`/simulation/ground_truth_world_markers`, and owns the evaluation-only
`map -> simulation_world_ground_truth` transform.

The alternative of hard-coding markers in the existing cuVSLAM launch was
rejected because SDF parsing, alignment, marker lifecycle, and tests are an
independent observability responsibility. The reference data never feeds
cuVSLAM, mapping, costmaps, or navigation.

## Files Changed

- `Foxglove Simulation.command`
- `tools/jetson_connection_web.py`
- `deployment/isaac_ros/isaac_ros_visual_slam_d435i_sim.launch.py`
- `deployment/isaac_ros/cuvslam_sim_observability/`
- `deployment/foxglove/README.md`
- `docs/FOXGLOVE_WORLD_OUTLINE_DESIGN.md`
- `docs/SIMULATION_HANDOFF.md`
- `reports/foxglove_mac_simulation_20260728.md`

The directory is not a Git repository, so no Git diff or commit is available.
The source/config behavior change is limited to the launcher, existing
operator GUI, and simulation-only cuVSLAM launch; the other files are records.

## Verification

Passed:

- Launcher Bash syntax check.
- GUI Python syntax check with bytecode redirected to `/tmp`.
- Foxglove installation detected at `/Applications/Foxglove.app`.
- Launcher selected `tseng@192.168.55.1:22`.
- Bridge listened only on Jetson `127.0.0.1:8765`.
- SSH listened only on Mac `127.0.0.1:8766`.
- WebSocket handshake returned HTTP 101 with
  `foxglove.websocket.v1`.
- Server advertised only the `connectionGraph` capability.
- Allowed simulation, D435i, TF, ground-truth, and cuVSLAM channels were
  advertised.
- Foxglove client connected and subscribed to `/tf`, `/tf_static`, and
  `/scan`.
- One `/scan` message was received with frame
  `slam_bot/laser_frame/lidar`.
- Stopping the tunnel removed both listeners and the launcher-owned bridge.
- Restart recreated both listeners and Foxglove reconnected.
- The GUI `Open Foxglove` API reused the active tunnel and completed in 0.2 s.
- The GUI work log recorded request, Jetson endpoint, tunnel reuse, and
  completion.
- Repeated GUI clicks bring the existing App to the foreground without adding
  another Foxglove subscriber or generating a WebSocket probe error.
- The GUI exposes one `Start simulation` entry point for D435i, cuVSLAM,
  Foxglove, and the low-rate monitor; separate Foxglove and cuVSLAM start
  buttons were removed.
- A full four-stage stop completed in 4.1 s.
- A cold four-stage start completed in 30.7 s and reached `vo_state=1`.
- A repeated start reused healthy components and completed in 9.3 s.
- The first post-start monitor sample reported `Healthy`, D435i 5/5 topics,
  scan/truth ready, and cuVSLAM tracking.
- `tf2_echo map slam_bot/laser_frame/lidar` returned a live transform after the
  simulation-only extrinsic was deployed.
- `/scan` was enabled in the Foxglove 3D panel and red wall contours remained
  visible after a full stop/start cycle.
- `cuvslam_sim_observability` built successfully in the Humble container.
- Four geometry and SDF parser tests passed in the Humble container.
- The packaged and active Baseline SDF SHA-256 values matched:
  `ec72783670920977ec5dc4e0cd1363f7aeb879e05401bd771c2bf3951b90a004`.
- `/simulation/ground_truth_world_markers` had exactly one publisher with
  reliable, transient-local QoS.
- One received MarkerArray contained seven wireframes: floor, four walls, and
  two feature boxes.
- `tf2_echo map simulation_world_ground_truth` returned the alignment
  transform and exactly one node owns it.
- The Foxglove bridge whitelist includes `/simulation/.*` and remains bound to
  Jetson localhost.
- The GUI monitor reported `World outline: ready` and overall `Healthy`.
- A repeated full start reached D435i ready, cuVSLAM `vo_state=1`, Foxglove
  ready, and monitor active in 16.2 seconds.

The first bridge attempt failed because empty ROS launch array arguments did
not produce valid parameter values. The launcher now uses
`capabilities:=[connectionGraph]` and never-match regex lists for disabled
client operations. Restart and cleanup passed after this correction.

Jetson rollback backup:

```text
/home/tseng/isaac_ros_ws/src/isaac_ros_visual_slam/isaac_ros_visual_slam/launch/isaac_ros_visual_slam_d435i_sim.launch.py.bak_20260728_1445
/home/tseng/isaac_ros_ws/src/isaac_ros_visual_slam/isaac_ros_visual_slam/launch/isaac_ros_visual_slam_d435i_sim.launch.py.bak_20260728_152026
```

Not yet tested:

- Exported `cuVSLAM Level 0` custom Foxglove layout; the world marker topic is
  available and must currently be enabled once in the 3D panel topic list.
- Long-duration bandwidth, memory, and thermal behavior.
- Full-rate stereo image viewing while cuVSLAM and landmark clouds are active.
- The independent mapping test world; this run used the Baseline world.

## Rollback

Use the GUI `Stop` button for the full workflow. For Foxglove only, close the
`Foxglove Simulation.command` Terminal or press Control-C. The launcher stops
its SSH tunnel and only the bridge instance it started.

To roll back the LiDAR transform, restore the timestamped Jetson backup over
the active launch file and restart cuVSLAM. Remove the
`cuvslam_sim_observability` source package only after the launch rollback. No
active Gazebo world, Nav2, production service, or real-robot workspace was
modified.

## Residual Risk

- Foxglove 2.57.0 is newer than bridge 0.7.2. The protocol handshake and
  current subscriptions work, but newer panel features may require a bridge
  update later.
- Stereo images and cuVSLAM landmark clouds can consume substantially more
  bandwidth than TF and LaserScan.
- `/ground_truth/odom` is evaluation-only and must not become a localization
  input.
