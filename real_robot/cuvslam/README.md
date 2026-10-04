# Real-robot cuVSLAM

This directory is the real-camera cuVSLAM workstream. It contains the
RealSense D435i launch boundary, hardware tools, calibration records, and
real-camera evidence.

Deployment target: the reviewed real-robot workspace only, after explicit
human approval. Do not place simulation worlds, Gazebo truth, or navigation
research here.

The macOS GUI accepts both the reorganized Jetson path
`/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/` and the pre-reorganization
legacy path `/home/tseng/isaac_ros_ws/tools/`. This keeps an unsynchronized
Jetson usable during the repository migration; the selected remote path is
written to the GUI work log. Its R2 live view retains the cuVSLAM XY odometry
path and can also display the `/map` occupancy grid, `/scan` points, and the
map-frame projected path when the A2M12 + `slam_toolbox` mapping runtime is
active on Jetson. Selecting `mapping（建圖）` in the GUI starts that runtime;
selecting `odom` keeps the existing R2 runner.

The Jetson runtime layout keeps the existing Isaac ROS workspace and separates
the workstreams below it without duplicating the large source, bag, or Docker
layers:

```text
/home/tseng/isaac_ros_ws/real_robot/cuvslam/
  launch/  config/  tools/  calibration/
/home/tseng/isaac_ros_ws/simulation/cuvslam/
  tools/  config/  data/
/home/tseng/isaac_ros_data/real_robot/cuvslam/
  experiments/  bags/  maps/  logs/  calibration/  runs/
```

The old top-level `tools/`, `config/`, `data/`, and
`isaac_ros_data/experiments` paths remain compatibility symlinks. New runtime
files belong under the canonical workstream paths.

## Experimental D435i + RPLIDAR A2M12 integration

The first 2D mapping integration is kept separate from the accepted R2 runner:

```text
D435i/cuVSLAM odometry-only -> odom -> base_link
RPLIDAR A2M12              -> /scan, frame_id=rplidar_link
slam_toolbox               -> map -> odom
```

The launch and profile are:

- `deployment/isaac_ros/isaac_ros_visual_slam_d435i_rplidar.launch.py`
- `config/rplidar/a2m12.yaml`
- `config/real_d435i_rplidar_slam_toolbox.yaml`
- `tools/run_real_d435i_rplidar.sh`

The launch uses replaceable `base_link` extrinsic placeholders for the first
stationary wiring test. The fixed A2M12 mount is now represented by the
software `base_link -> rplidar_link` `lidar_yaw` argument, forwarded by the GUI
and recorded in each session; its default remains `0.0` until a labelled motion
test confirms the orientation. Do not use this for motion or mapping accuracy
claims until the camera and A2M12 transforms are measured. The Slamtec ROS 2
`sllidar_ros2` package has now been built in the Jetson target workspace and the
A2M12 stationary data path has passed its first gate.

On the Jetson, use the isolated runner from the canonical path:

```bash
/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh start
/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh status
/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh telemetry
/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh quality
/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh preflight
/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh record_start RUN_NAME light
/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh record_stop RUN_NAME
/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh stop
```

The macOS GUI uses these same telemetry, quality, recording, and cleanup
actions when `mapping（建圖）` is selected. Mapping recordings are stored under
`/home/tseng/isaac_ros_data/real_robot/cuvslam/experiments/real_d435i_rplidar/`.

The `stop` action first calls the A2M12 `/stop_motor` service, then stops the
whole mapping container including `sllidar_node`; the separate RViz observation
container is left untouched.
The runner does not start Nav2, `/cmd_vel`, or robot motion. Calibrate and
replace `camera_x/y/z/yaw` and `lidar_x/y/z/yaw` before any movement test.

## Diagnostic LiDAR odometry + EKF shadow

The mapping runner now has an optional diagnostic estimator stack. The default
`cuvslam` stack is unchanged. Selecting `ekf_shadow` starts:

```text
RPLIDAR /scan -> rf2o_laser_odometry -> /lidar/odom
cuVSLAM /visual_slam/tracking/odometry ─┐
                                       ├-> robot_localization -> /odometry/filtered_shadow
LiDAR /lidar/odom                      ┘
```

Both additional outputs are diagnostic topics only. `rf2o` and the EKF have
TF publication disabled, so cuVSLAM remains the sole `odom -> base_link`
owner and slam_toolbox remains the sole `map -> odom` owner. The GUI compares
the relative pose of the three streams and records the result without
rewriting the formal TF tree.

The work record reports two separate meanings of EKF contribution:

- `介入程度（軌跡推估）`: geometric projection of the EKF pose between the
  cuVSLAM and LiDAR trajectories; this is not a Kalman gain percentage.
- `EKF 內部診斷`: fields actually published by `robot_localization` on
  `/diagnostics`, such as processed/rejected/dropped measurements and
  frequency. Missing fields are shown as unavailable. The standard ROS
  diagnostic stream does not expose one human-readable per-sensor gain
  percentage.

The extra ROS packages are external Jetson dependencies and are not vendored
in this repository. After the mapping container is running, check them with:

```bash
/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh preflight
```

For a direct run, the optional seventh `start` argument is the fixed-mount
LiDAR yaw in radians. The three following optional arguments are the fixed
mount translation `x`, `y`, and `z` in metres. The environment form is also
supported:

```bash
CUVSLAM_LIDAR_X=0.0 \
CUVSLAM_LIDAR_Y=0.0 \
CUVSLAM_LIDAR_Z=0.0 \
CUVSLAM_LIDAR_YAW=0.0 \
/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh \
  start imu 640x360x30 official standard cuvslam 0.0 1 0.0 0.0 0.0
```

The direct example above uses zero translation as a safe placeholder. The
current Mac GUI mapping setting uses the user-provided candidate
`x=0.0, y=-0.10, z=0.0` for the fixed mount, with the LiDAR on the camera's
right. This remains a hardware test candidate rather than a verified
calibration result. The arguments change only the software static TF
parameters; they do not move the sensor and do not change TF ownership. Do
not set `pi` merely because the sensor appears visually reversed; confirm it
with the labelled motion test.

Only after that check passes should the GUI estimator be set to
`LiDAR + EKF shadow（診斷，不接管 TF）`, or the equivalent runner invocation:

```bash
/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh \
  start imu 640x360x30 official standard ekf_shadow
```

This first implementation is not a production fusion promotion. It does not
change `/cmd_vel`, Nav2, calibrated extrinsics, map ownership, or the formal
odom TF. Promotion requires separate pure-turn, straight-motion, stop, and
timestamp/frame checks with one explicitly approved TF owner.

The project governance entry points remain at the repository root:
`AGENTS.md`, `ENGINEERING_GUIDELINES.md`, and `PROJECT_HANDOFF.md`.
