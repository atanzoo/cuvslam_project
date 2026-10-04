# Mac Foxglove Simulation Viewer

Updated: 2026-07-28 (Asia/Taipei)

## Objective

Open the Jetson ROS domain 43 simulation in the Foxglove desktop app on the
Mac without exposing a WebSocket port on the Jetson network interfaces.

This is an observability-only workflow. It does not start Gazebo, D435i,
cuVSLAM, robot motion, mapping, or Nav2.

## Selected Design

The user selected Foxglove over RViz for the Mac viewer. Two bridge placements
were evaluated:

### A. Humble Isaac ROS container selected

- Uses the already installed `foxglove_bridge 0.7.2`.
- The container uses host networking and can see ROS domain 43.
- The bridge binds only to Jetson `127.0.0.1:8765`.
- Mac `127.0.0.1:8766` forwards to the bridge through SSH.
- No package installation or Foxy source build is required.

### B. Foxy host source build rejected

- Foxglove documentation states ROS 2 distributions older than Humble require
  a source build.
- This would add a dependency and maintenance path to the simulation host
  without improving the Mac viewing workflow.

## Security And Ownership

- The bridge address is `127.0.0.1`; it is not reachable directly from the
  Jetson LAN or USB network.
- Foxglove exposes only the read-only connection graph capability. Client
  publishing, services, parameters, and asset fetching use never-match
  whitelists and are disabled.
- The topic whitelist contains only simulation sensors, TF, clock, ground
  truth, `/simulation/*`, and `/visual_slam/*`.
- `/ground_truth/odom` is visible for evaluation only and does not feed
  cuVSLAM.
- `/simulation/ground_truth_world_markers` is a visualization-only reference.
  It must not feed cuVSLAM, localization, mapping, costmaps, or navigation.
- The launcher stops only a bridge that it started. It never stops Gazebo,
  the ROS bridge, or cuVSLAM.

## Operation

The normal entry point is the `Start simulation` button in the Mac control
GUI. It now starts the dense v3 simulation configuration, in order:

1. Gazebo, the ROS bridge, and D435i topics.
2. cuVSLAM and waits for `vo_state=1`.
3. The Foxglove bridge, SSH tunnel, and desktop App.
4. The SSH low-rate simulation monitor.

The button uses the dense observable world
`cuvslam_mapping_observable_640x360_range2p5_dense_turn.sdf`, stereo-only
cuVSLAM, mapping visualization, and `enable_imu_fusion=false`. Wheel odometry
is not started or used.

The GUI `Stop` button stops the same four layers in reverse ownership order.
Repeated `Start simulation` requests reuse healthy running components.

The standalone fallback entry point is:

```text
/Users/tsengpochien/Desktop/cuvslam_project/Foxglove Simulation.command
```

The launcher:

1. Selects the first working Jetson SSH endpoint.
2. Starts the read-only Foxglove bridge if needed.
3. Creates the SSH tunnel on Mac port 8766.
4. Opens `/Applications/Foxglove.app` with
   `ws://127.0.0.1:8766`.

Keep the launcher Terminal open while viewing. Press Control-C or close that
Terminal to end the tunnel.

## What The 3D View Shows

Foxglove displays ROS data, not the Gazebo renderer or the world's visual
materials. The Baseline 3D panel currently shows:

- cuVSLAM camera TF and trajectory frames;
- `/scan` as red LaserScan wall and obstacle points;
- `/simulation/ground_truth_world_markers` as the known Baseline geometry;
- selected cuVSLAM paths or clouds when those topics publish data.

The world reference is published by the simulation-only
`cuvslam_sim_observability` package. It parses static box collisions from the
packaged Baseline SDF and publishes seven wireframes:

- gray: floor;
- cyan: four walls;
- yellow: two feature boxes.

The markers use frame `simulation_world_ground_truth`. At startup, the node
uses synchronized ground-truth odometry and the cuVSLAM camera transform to
publish one evaluation-only transform:

```text
map -> simulation_world_ground_truth
```

This aligns the known world with cuVSLAM's local `map` frame without making
ground truth a localization input. In the Foxglove 3D panel, enable
`/simulation/ground_truth_world_markers` under Topics. Compare the red `/scan`
returns and cuVSLAM trajectory against the cyan/yellow reference wireframes.

The Baseline SDF did not publish the LiDAR frame into ROS TF. The
simulation-only cuVSLAM launch now publishes:

```text
camera_infra1_optical_frame -> slam_bot/laser_frame/lidar
```

The transform is derived from the Baseline SDF sensor poses and optical-frame
axis convention. This lets Foxglove place `/scan` in the `map` tree. An empty
plot can be normal while the robot is stationary; `/cmd_vel` remains zero
until a motion command is published.

The official Foxglove ROS 2 guide recommends Foxglove Bridge for live ROS 2
visualization:

```text
https://docs.foxglove.dev/docs/getting-started/frameworks/ros2
```

## Initial Panels

The current desktop layout has `/scan` enabled and the setting survived a full
stop/start cycle. A later exported layout named `cuVSLAM Level 0` should use:

| Panel | Source |
|---|---|
| Image | `/d435i/infra1/image_rect_raw` |
| Image | `/d435i/infra2/image_rect_raw` |
| 3D | `/tf`, `/tf_static`, `/scan`, `/simulation/ground_truth_world_markers`, cuVSLAM paths and landmarks |
| Raw Messages | `/visual_slam/status` |
| Plot | `/visual_slam/tracking/odometry` pose fields |
| Plot | `/ground_truth/odom` pose fields, clearly marked evaluation only |

Foxglove layout JSON is an opaque application export format. Create the layout
in the installed app and export it through the Layout menu rather than
hand-authoring JSON.

Official layout operation:

```text
https://docs.foxglove.dev/docs/visualization/layouts
```

## Rollback

Close the launcher Terminal. If its bridge does not exit cleanly, verify the
exact process before stopping it:

```bash
ssh tseng@192.168.55.1 \
  docker exec isaac_ros_dev-aarch64-container \
  pgrep -af '[f]oxglove_bridge_launch.xml'
```

No ROS package, simulator world, cuVSLAM launch, or Jetson system service is
modified by this workflow.
