# Isolated Isaac ROS Environment

This directory defines the isolated Isaac ROS deployment boundary for the
Jetson. It must not source or mount the existing Foxy workspaces.

Host paths:

- Isaac ROS workspace: `/home/tseng/isaac_ros_ws`
- Simulation data: `/home/tseng/isaac_ros_ws/simulation/cuvslam/data`
- Real D435i data: `/home/tseng/isaac_ros_data/real_robot/cuvslam`
- Legacy data aliases are retained for compatibility only.
- Existing Foxy workspace: `/home/tseng/slam_ws` (never mounted)
- Existing simulation workspace: `/home/tseng/jetson_slam_ws` (never mounted)

Isolation rules:

- Run Isaac ROS only through `run.sh`.
- Use a dedicated `ROS_DOMAIN_ID` (default `42`).
- Do not source `/home/tseng/slam_ws/install/setup.bash` in the Isaac ROS shell.
- Do not mount either existing Foxy workspace into the container.
- Do not start production Nav2, AMCL, or LiDAR ICP from this container.
- The image reference is deliberately required in `isaac_ros.env`; it must be
  selected only after the JetPack/ROS compatibility decision is recorded.

The current Jetson reports JetPack 5.1.3, CUDA 11.4, and ROS 2 Foxy. This
workspace uses the reviewed legacy Isaac ROS 2.1 source route and the verified
local image `isaac_ros_dev-aarch64:fixed`.

## Setup

1. Use the deployed `/home/tseng/isaac_ros.env` with the verified image.
2. Ensure the `tseng` user can run Docker without sudo, then reconnect SSH.
3. Run `/home/tseng/isaac_ros_deployment/run.sh shell`.

The container preflight milestone is complete: ROS version, CUDA visibility,
GPU/runtime setup, package installation, source build, and USB-bus visibility
were verified. RealSense device enumeration and firmware verification remain
hardware bring-up steps.

For simulation/replay, use ROS domain `43` and follow
`docs/SIMULATION_HANDOFF.md`. The verified container default is domain `42`;
override the domain explicitly for simulation so replay traffic remains
isolated.

## Simulation Sensor Adapter

`cuvslam_sim_sensor_adapter` is a simulation-only sensor-normalization package.
It preserves the Gazebo bridge's raw right CameraInfo and publishes a cuVSLAM
input copy with the declared stereo baseline encoded in `P[3]`.

```text
/d435i/infra2/camera_info
  -> /cuvslam/input/infra2/camera_info
```

The adapter must not be deployed to the real-robot workspace. The real D435i
driver remains responsible for its own calibrated CameraInfo.

## Simulation TF Architecture

The simulation launch follows the native NVIDIA `base_frame` mechanism:

```text
map -> odom -> base_link -> sensors
```

`isaac_ros_visual_slam` is the sole publisher of `map -> odom` and
`odom -> base_link`. Static transform publishers in the simulation launch own
the locked `base_link -> sensor` extrinsics. The left physical camera frame,
`camera_infra1_frame`, is supplied through `input_left_camera_frame`; the
optical frame remains the image frame only.

The right camera's 0.05 m stereo baseline continues to come exclusively from
the adapted right CameraInfo. `input_right_camera_frame` is intentionally empty
to avoid a second stereo-extrinsic authority.

This architecture is not yet accuracy-qualified. The three-run motion gate in
`docs/CUVSLAM_TF_CONTRACT_FIX_DESIGN.md` remains mandatory.
