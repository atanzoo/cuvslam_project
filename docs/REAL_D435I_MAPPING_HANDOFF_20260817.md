# Real D435i cuVSLAM Mapping Handoff

Updated: 2026-08-17 (Asia/Taipei)

> This is the original real-camera R0–R3 handoff and remains useful for the
> procedure. The current real D435i result, IMU-fusion decision, R2 metrics and
> correction record are maintained in
> `docs/CUVSLAM_CURRENT_STATUS_20260825.md` and
> `logs/real_d435i_quality/20260825_r2_correction_log.md`.

This document is the entry point for the next task: measuring a real Intel
RealSense D435i with cuVSLAM on the Jetson AGX Orin. It separates real-camera
evidence from the Gazebo simulation baseline. Do not treat simulation truth,
wheel odometry, or Foxglove world overlays as real-camera measurements.

## 1. Current Objective

Start the first controlled real-camera mapping experiment with:

```text
Real D435i stereo + optional D435i IMU
        -> Jetson AGX Orin / Isaac ROS Visual SLAM
        -> cuVSLAM visual map, trajectory, and tracking health
        -> rosbag2 and Foxglove observability
```

The immediate goal is to prove the real sensor data path and produce one
repeatable visual map. Navigation, Nav2, LiDAR fusion, obstacle avoidance, and
real-robot velocity control are out of scope for this first run.

## 2. What Is Complete

### 2.1 Jetson and Isaac ROS baseline

- Jetson AGX Orin target identified.
- JetPack 5.1.3 / Jetson Linux R35.5.0 recorded.
- Host ROS 2 Foxy and Isaac ROS container ROS 2 Humble recorded.
- Isaac ROS Visual SLAM 2.1.0 / cuVSLAM 11.4 installed and verified in the
  isolated container.
- NVIDIA release-2.1 official stereo control passed: 725 left images
  produced 725 odometry and 725 status messages, all with `vo_state=1`.
- The Mac can reach Jetson through USB SSH at `192.168.55.1`.
- The Jetson currently also has Wi-Fi connectivity through `wlan0`; the last
  verified address was `172.20.10.2` with working external ping and DNS.

Primary files:

- `deployment/isaac_ros/README.md`
- `deployment/isaac_ros/current_host_inventory.md`
- `deployment/isaac_ros/run.sh`
- `docs/SIMULATION_HANDOFF.md`
- `PROJECT_HANDOFF.md`

### 2.2 cuVSLAM frame architecture

The simulation was migrated to the NVIDIA-native output architecture:

```text
map -> odom -> base_link -> camera_link -> D435i frames
                              |-> camera_infra1_frame
                              |-> camera_infra2_frame
                              |-> camera_imu_frame
                              `-> lidar_link
```

cuVSLAM owns the dynamic `map -> odom` and `odom -> base_link` transforms.
Static sensor extrinsics are external static transforms. The real-camera test
must verify that the D435i driver publishes a compatible frame tree before
using any simulation-only aliases.

Authoritative contract:

- `docs/sensor_and_frame_contract.md`
- `docs/CUVSLAM_TF_CONTRACT_FIX_DESIGN.md`
- `docs/STATIC_TF_TARGET_VALIDATION_DESIGN.md`

### 2.3 Stereo geometry correction

The simulation right CameraInfo adapter encodes the 0.05 m stereo baseline:

```text
P[3] = -fx * baseline
```

This is a simulation-only normalization. The real D435i driver must remain the
authority for its factory-calibrated CameraInfo; do not apply the simulation
adapter to real data.

Implementation:

- `deployment/isaac_ros/cuvslam_sim_sensor_adapter/`
- `deployment/isaac_ros/isaac_ros_visual_slam_d435i_sim.launch.py`
- `tools/validate_stereo_geometry.py`
- `tools/validate_sim_camera_intrinsics.py`
- `docs/CUVSLAM_STAGE1_INTRINSICS_AUDIT_DESIGN.md`
- `docs/CUVSLAM_STAGE2_STEREO_GEOMETRY_DESIGN.md`

### 2.4 Simulation and observability

The simulation can start Gazebo, the D435i topics, cuVSLAM, Foxglove Bridge,
SSH forwarding, and the low-rate monitor from the Mac GUI. Foxglove is
observability-only and does not provide camera truth or localization input.

Main entry points:

- `Jetson Connection.command`
- `tools/jetson_connection_gui.py`
- `Foxglove Simulation.command`
- `deployment/foxglove/README.md`
- `deployment/slam_gazebo/launch/gz_bridge_portable.launch.py`
- `deployment/isaac_ros/isaac_ros_visual_slam_d435i_sim.launch.py`

## 3. Simulation Evidence And Limits

### Accepted simulation evidence

- RealSense-like Gazebo stereo topics reach cuVSLAM.
- cuVSLAM reaches `vo_state=1` and publishes odometry/pose/status.
- The official NVIDIA stereo input contract is functional.
- Static TF ownership and the `base_link` output architecture are defined.
- Dense near-field visual geometry can make controlled simulated turns pass in
  two independent stereo-only cold starts.

### Important failed or limited results

- The sparse simple world produced severe translation and map-geometry errors.
- Straight mapping generated populated landmark clouds, but their geometry was
  not accepted as a reliable map.
- A longer `1 m -> 90 deg -> 0.5 m` route kept tracking active but failed the
  pose gate and contained extreme landmark outliers.
- Simulated IMU fusion was not accepted in the tested corner A/B; keep it
  disabled until real-camera IMU timing, frame direction, and calibration are
  independently checked.
- `/ground_truth/odom` in the simulation is wheel odometry and is prohibited as
  cuVSLAM input or physical truth. Native Gazebo pose is evaluation-only.

Key reports:

- `reports/cuvslam_d435i_sim_integration_20260727.md`
- `reports/cuvslam_observable_world_repeatability_20260731.md`
- `reports/cuvslam_first_observable_straight_mapping_20260731.md`
- `reports/cuvslam_triangulation_and_roundtrip_20260731.md`
- `reports/cuvslam_in_place_turn_pose_fix_20260731.md`
- `reports/cuvslam_initial_dense_v3_mapping_20260731.md`
- `reports/cuvslam_d435i_imu_corner_audit_20260731.md`
- `reports/cuvslam_stage4_official_input_contract_20260730.md`

## 4. Real D435i Readiness Gap

The current documents explicitly state that real-device enumeration and
firmware verification remain hardware bring-up steps. Before mapping, record:

1. D435i serial number and firmware version.
2. Actual left/right image topics and encodings.
3. Actual CameraInfo `K`, `D`, `R`, `P`, resolution, and `frame_id`.
4. Actual image timestamps and left/right timestamp skew.
5. Actual IMU topics, rate, frame ID, and gyro/accelerometer units.
6. Actual TF tree and publisher ownership.
7. Isaac ROS image QoS compatibility and `rectified_images` setting.
8. Jetson power mode, temperature, GPU load, dropped frames, and storage.

Do not copy simulated frame aliases, simulated baseline patches, simulated
world-outline publishers, or simulated LiDAR transforms into the real-robot
workspace.

## 5. First Real-Camera Test Sequence

### Stage R0: Device-only bring-up

With the D435i stationary and no robot motion:

```bash
realsense-viewer
rs-enumerate-devices
ros2 topic list
ros2 topic hz /camera/infra1/image_rect_raw
ros2 topic hz /camera/infra2/image_rect_raw
ros2 topic hz /camera/imu
ros2 run tf2_tools view_frames
```

Use the actual deployed topic names; the commands above are a checklist, not
an assumption that the real driver uses the simulation `/d435i/*` namespace.

Record the output in a new experiment directory under `logs/` or `datasets/`
with the device serial and timestamp.

### Stage R1: Static stereo contract

Run the camera stationary for at least 30 seconds and verify:

- left/right images are non-empty and changing when the scene changes;
- left/right CameraInfo resolutions match the images;
- left/right `K/P` are internally consistent;
- the right projection matrix contains the factory stereo baseline;
- left/right image timestamps are near-synchronous;
- no duplicate publisher owns the same TF edge;
- IMU timestamps advance and the stationary gyro mean is near zero.

The first formal real run should use the thesis configuration with IMU fusion:

```text
enable_imu_fusion = true
mapping = true
debug mode = false
```

### Stage R2: Controlled straight mapping

Use a textured, static environment with visible side features. Avoid a blank
wall and avoid turning during the first run. The operator should manually move
the camera/robot slowly for a short straight segment, stop, and repeat from
the same start area.

Record at minimum:

- left/right images and CameraInfo;
- IMU;
- `/tf` and `/tf_static`;
- `/visual_slam/status`;
- `/visual_slam/tracking/odometry`;
- `/visual_slam/tracking/vo_pose`;
- `/visual_slam/tracking/slam_path`;
- cuVSLAM landmark/map topics when enabled;
- system resource and thermal data.

Do not judge success from `vo_state=1` alone. Compare repeatability, tracking
loss, image timing, stereo geometry, trajectory smoothness, and map output.

### Stage R3: Repeatability and turn

Only after R2 is clean, repeat the same route three times. Then add one small
controlled turn while keeping IMU fusion enabled. A stereo-only run may still
be used as a separately logged diagnostic comparison, but it is not the formal
thesis baseline.

### Real-camera lighting qualification update (2026-08-18)

The real R2 GUI now keeps IMU fusion selectable and adds an image-side lighting
diagnostic for both infrared streams: mean/std, dark and saturated pixel
percentages, gradient proxy, image rate, and timestamp gap. Motion recordings
save these values with the odometry metrics under `logs/real_d435i_quality/`.

The current formal profile is:

```text
IMU fusion = true
denoise_input_images = true
auto exposure = true
IR projector = on
profile = 640x360x30
```

This is called `official` in `tools/run_real_d435i_r2.sh`. `robust` keeps
auto exposure and denoising but disables the IR projector for diagnostic
comparison; `low_light` is retained as an alias of `official`.
Fixed manual exposure was tested and rejected because it reduced image
strength/rate and did not produce stable cuVSLAM odometry. A high saturation
warning must therefore be treated as a scene-lighting condition to measure and
correct, not as permission to apply a fixed odometry scale multiplier.

## 6. Real Experiment Bundle

Each run should have a unique directory containing:

```text
metadata.yaml
rosbag2/
tf_frames.pdf or tf_frames.yaml
camera_info.txt
device_info.txt
cuVSLAM_parameters.yaml
system_stats.log
operator_notes.md
evaluation.md
```

`metadata.yaml` must state hardware, serial, firmware, JetPack, ROS domain,
container image, topic remappings, resolution, frame IDs, IMU setting, and the
exact test route.

## 7. File Map For The Next Conversation

| Need | Start here |
|---|---|
| Current project state | `PROJECT_HANDOFF.md` |
| Mandatory engineering process | `ENGINEERING_GUIDELINES.md` |
| Simulation/replay boundary | `docs/SIMULATION_HANDOFF.md` |
| Frame definitions and ownership | `docs/sensor_and_frame_contract.md` |
| NVIDIA-native cuVSLAM launch | `deployment/isaac_ros/isaac_ros_visual_slam_d435i_sim.launch.py` |
| Jetson container setup | `deployment/isaac_ros/README.md`, `deployment/isaac_ros/run.sh` |
| Mac GUI and one-button workflow | `tools/jetson_connection_gui.py`, `Jetson Connection.command` |
| Foxglove tunnel/viewer | `deployment/foxglove/README.md`, `Foxglove Simulation.command` |
| Stereo validation tools | `tools/validate_stereo_geometry.py`, `tools/evaluate_stereo_bag.py` |
| Odometry evaluation | `tools/evaluate_odometry_bag.py`, `tools/evaluate_trajectory_geometry_bag.py` |
| TF evaluation | `tools/evaluate_frame_contract_bag.py`, `tools/validate_static_tf_targets.py` |
| Existing experiment evidence | `reports/` |
| Camera and calibration records | `calibration/README.md`, `calibration/` |

## 8. Explicit Non-Goals For The Next Task

- Do not modify the production LiDAR/Nav2 workspace under
  `/Applications/slam_v2`.
- Do not deploy simulation-only adapters to the real-robot workspace.
- Do not use Gazebo truth, wheel odometry, or a post-processing scale factor to
  improve a real-camera result.
- Do not start Nav2 or obstacle avoidance before real visual mapping and
  localization have a measured interface.
- Do not modify TF or CameraInfo based on a single real run; first collect the
  static contract and three repeatable measurements.

## 9. Suggested Opening Prompt For A New Conversation

```text
請先閱讀：
/Users/tsengpochien/Desktop/cuvslam_project/ENGINEERING_GUIDELINES.md
/Users/tsengpochien/Desktop/cuvslam_project/PROJECT_HANDOFF.md
/Users/tsengpochien/Desktop/cuvslam_project/docs/REAL_D435I_MAPPING_HANDOFF_20260817.md
/Users/tsengpochien/Desktop/cuvslam_project/docs/sensor_and_frame_contract.md

我現在已取得實體 Intel RealSense D435i，請先不要修改 TF、CameraInfo 或
cuVSLAM 參數。先完成 R0/R1 硬體與資料契約盤點：確認序號、韌體、左右影像、
CameraInfo K/P、時間戳、IMU、TF tree、QoS 與 Jetson 資源狀態。請把所有證據
寫入一個新的實驗資料夾，再回報是否可以開始 R2 直行實測建圖。
```
