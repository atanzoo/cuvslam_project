# Current Jetson Host Inventory

Recorded: 2026-07-27 Asia/Taipei

## Host

- Model: Jetson AGX Orin Developer Kit
- JetPack: 5.1.3-b29
- Jetson Linux: R35.5.0
- Kernel: 5.10.192-tegra
- OS: Ubuntu 20.04.6 LTS
- CUDA: 11.4 packages installed
- ROS 2: Foxy
- Docker: 26.1.3
- NVIDIA Docker runtime: registered

## Existing workspaces

- `/home/tseng/slam_ws`: 141 MB
- `/home/tseng/jetson_slam_ws`: 271 MB
- Existing workspaces are protected and are not mounted by the Isaac ROS launcher.

## Isaac ROS isolation

- Workspace: `/home/tseng/isaac_ros_ws`
- Simulation data: `/home/tseng/isaac_ros_ws/simulation/cuvslam/data`
- Real D435i data: `/home/tseng/isaac_ros_data/real_robot/cuvslam`
- Legacy data aliases remain for compatibility.
- ROS domain: `42`
- Launcher: `/home/tseng/isaac_ros_deployment/run.sh`
- Image: `isaac_ros_dev-aarch64:fixed`

## Backup

- Local backup: `backups/jetson_baseline_20260724.tar.gz`
- Contents: existing `slam_ws` and `jetson_slam_ws`
- The backup excludes workspace `log` directories.

## Notes

- A Foxy `/usb_dds_test` monitor was running during inventory and was not stopped.
- Container: `isaac_ros_dev-aarch64-container`
- Container ROS: Humble with `ROS_DOMAIN_ID=42`
- Source build: 36 packages finished, including `isaac_ros_visual_slam`
- Visual SLAM install prefix: `/workspaces/isaac_ros-dev/install/isaac_ros_visual_slam`
- CUDA 11.4 and JetPack host libraries are mounted into the container.
- USB bus is visible inside the container.
- RealSense device enumeration and firmware are not yet verified in the current
  hardware session.
- Simulation/replay uses ROS domain `43`; the verified Isaac ROS preflight
  default remains domain `42`.
- No host package, existing ROS workspace, or production process was changed.
