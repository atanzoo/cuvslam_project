#!/usr/bin/env bash
set -euo pipefail

CONTAINER="real_d435i_rplidar"
# The shadow estimator image contains the pinned offline GeographicLib layer
# required by robot_localization.  Allow a deliberate override for rollback.
IMAGE="${ISAAC_ROS_IMAGE:-isaac_ros_dev-aarch64:ekf-shadow}"
WORKSPACE="/home/tseng/isaac_ros_ws"
DATA_DIR="/home/tseng/isaac_ros_data/real_robot/cuvslam"
RUN_DIR="${DATA_DIR}/experiments/real_d435i_rplidar"
RPLIDAR_BAG_PID_FILE="/tmp/real_d435i_rplidar_bag.pid"
RPLIDAR_LIGHT_MIN_FREE_MB=512
RPLIDAR_FULL_MIN_FREE_MB=4096
RPLIDAR_RECORDING_RESERVE_MB=2048

light_record_topics=(
  /map
  /scan_raw
  /scan
  /tf
  /tf_static
  /camera/imu
  /visual_slam/status
  /visual_slam/tracking/odometry
  /lidar/odom
  /fusion/visual_odom
  /fusion/lidar_odom
  /fusion/odom_contract_diagnostics
  /odometry/filtered_shadow
  /diagnostics
  /visual_slam/tracking/vo_pose
  /visual_slam/tracking/slam_path
)

full_record_topics=(
  /map
  /scan_raw
  /scan
  /tf
  /tf_static
  /camera/infra1/image_rect_raw
  /camera/infra1/camera_info
  /camera/infra1/metadata
  /camera/infra2/image_rect_raw
  /camera/infra2/camera_info
  /camera/infra2/metadata
  /camera/imu
  /visual_slam/status
  /visual_slam/tracking/odometry
  /lidar/odom
  /fusion/visual_odom
  /fusion/lidar_odom
  /fusion/odom_contract_diagnostics
  /odometry/filtered_shadow
  /diagnostics
  /visual_slam/tracking/vo_pose
  /visual_slam/tracking/slam_path
  /visual_slam/vis/observations_cloud
  /visual_slam/vis/landmarks_cloud
  /visual_slam/vis/pose_graph_nodes
  /visual_slam/vis/pose_graph_edges
  /visual_slam/vis/loop_closure_cloud
)

case "${1:-status}" in
  start)
    mode="${2:-imu}"
    profile="${3:-640x360x30}"
    lighting="${4:-official}"
    diagnostic_mode="${5:-standard}"
    estimator_stack="${6:-cuvslam}"
    # Keep the physical A2M12 mount fixed; this is the software TF yaw from
    # base_link to rplidar_link.  The explicit argument is used by the GUI,
    # while the environment fallback keeps direct Jetson invocation simple.
    lidar_yaw="${7:-${CUVSLAM_LIDAR_YAW:-0.0}}"
    emitter_override="${8:-profile}"
    # Translation is appended after the existing emitter argument to keep the
    # previous positional interface backward-compatible.  Until measured,
    # these remain zero placeholders and must not be treated as calibrated.
    lidar_x="${9:-${CUVSLAM_LIDAR_X:-0.0}}"
    lidar_y="${10:-${CUVSLAM_LIDAR_Y:-0.0}}"
    lidar_z="${11:-${CUVSLAM_LIDAR_Z:-0.0}}"
    scan_tf_gate="${12:-${CUVSLAM_SCAN_TF_GATE:-true}}"
    numeric_pattern='^[-+]?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$'
    validate_lidar_value() {
      local name="$1"
      local value="$2"
      local unit="$3"
      if ! [[ "${value}" =~ ${numeric_pattern} ]]; then
        echo "${name} must be a numeric ${unit} value; got: ${value}" >&2
        exit 2
      fi
    }
    validate_lidar_value lidar_x "${lidar_x}" meter
    validate_lidar_value lidar_y "${lidar_y}" meter
    validate_lidar_value lidar_z "${lidar_z}" meter
    validate_lidar_value lidar_yaw "${lidar_yaw}" radian
    case "${scan_tf_gate}" in
      true|false) ;;
      *) echo "scan TF gate must be true or false; got: ${scan_tf_gate}" >&2; exit 2 ;;
    esac
    case "${mode}" in
      stereo) fusion=false ;;
      imu) fusion=true ;;
      *) echo "mode must be stereo or imu" >&2; exit 2 ;;
    esac
    case "${profile}" in
      640x360x30) driver_profile="640,360,30" ;;
      848x480x30) driver_profile="848,480,30" ;;
      *) echo "profile must be 640x360x30 or 848x480x30" >&2; exit 2 ;;
    esac
    case "${lighting}" in
      official|low_light)
        denoise=true; emitter=1; auto_exposure=true; exposure=8500; gain=16 ;;
      low_light_motion)
        # Experimental motion profile: keep the projector on while limiting
        # manual exposure to reduce blur.  Validate brightness on the target
        # camera before using this as a formal operating profile.
        denoise=true; emitter=1; auto_exposure=false; exposure=10000; gain=24 ;;
      baseline)
        denoise=false; emitter=1; auto_exposure=true; exposure=8500; gain=16 ;;
      robust)
        denoise=true; emitter=0; auto_exposure=true; exposure=8500; gain=16 ;;
      high_contrast)
        denoise=true; emitter=0; auto_exposure=false; exposure=8000; gain=16 ;;
      *) echo "lighting must be official, baseline, robust, low_light, low_light_motion, or high_contrast" >&2; exit 2 ;;
    esac
    case "${emitter_override}" in
      profile) ;;
      on) emitter=1 ;;
      off) emitter=0 ;;
      *) echo "emitter override must be profile, on, or off" >&2; exit 2 ;;
    esac
    case "${diagnostic_mode}" in
      standard) observations_view=false ;;
      features) observations_view=true ;;
      *) echo "diagnostic mode must be standard or features" >&2; exit 2 ;;
    esac
    case "${estimator_stack}" in
      cuvslam)
        enable_lidar_odom=false
        enable_ekf_shadow=false
        ;;
      ekf_shadow)
        enable_lidar_odom=true
        enable_ekf_shadow=true
        ;;
      *)
        echo "estimator stack must be cuvslam or ekf_shadow" >&2
        exit 2
        ;;
    esac
    if docker ps --format '{{.Names}}' | grep -Fxq "real_d435i_r2" || \
       docker ps --format '{{.Names}}' | grep -Fxq "real_d435i_r1p5_axis_gui" || \
       docker ps --format '{{.Names}}' | grep -Fxq "experimental_candidate_localization"; then
      echo "another D435i runtime is active; stop R1.5/R2 before starting integration" >&2
      exit 3
    fi
    if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
      echo "already_running container=${CONTAINER}"
      exit 0
    fi
    mkdir -p "${RUN_DIR}"
    docker run -d --rm --init --privileged \
      --name "${CONTAINER}" \
      --network host --ipc host --runtime nvidia \
      --env ROS_DOMAIN_ID=42 \
      --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      --env NVIDIA_VISIBLE_DEVICES=all \
      --env NVIDIA_DRIVER_CAPABILITIES=all \
      --volume "${WORKSPACE}:/workspaces/isaac_ros-dev" \
      --volume "${DATA_DIR}:/workspaces/isaac_ros_data" \
      --volume /dev:/dev \
      --volume /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h:/usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h:ro \
      --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11:ro \
      --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11:ro \
      --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10:ro \
      --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10:ro \
      --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so:ro \
      --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4:ro \
      --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1:ro \
      --volume /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
      --volume /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api:ro \
      --volume /tmp/argus_socket:/tmp/argus_socket \
      --workdir /workspaces/isaac_ros-dev \
      --entrypoint /bin/bash "${IMAGE}" \
      -lc "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_d435i_rplidar.launch.py enable_imu_fusion:=${fusion} infra_profile:=${driver_profile} denoise_input_images:=${denoise} emitter_enabled:=${emitter} enable_auto_exposure:=${auto_exposure} exposure:=${exposure} gain:=${gain} enable_observations_view:=${observations_view} enable_lidar_odom:=${enable_lidar_odom} enable_ekf_shadow:=${enable_ekf_shadow} enable_scan_tf_gate:=${scan_tf_gate} lidar_x:=${lidar_x} lidar_y:=${lidar_y} lidar_z:=${lidar_z} lidar_yaw:=${lidar_yaw}"
    echo "started container=${CONTAINER} mode=${mode} profile=${profile} lighting=${lighting} emitter_override=${emitter_override} diagnostics=${diagnostic_mode} estimator=${estimator_stack} denoise=${denoise} emitter=${emitter} auto_exposure=${auto_exposure} exposure=${exposure} gain=${gain} observations_view=${observations_view} lidar_odom=${enable_lidar_odom} ekf_shadow=${enable_ekf_shadow} scan_tf_gate=${scan_tf_gate} lidar_x=${lidar_x} lidar_y=${lidar_y} lidar_z=${lidar_z} lidar_yaw=${lidar_yaw}"
    ;;
  stop|cleanup)
    # A2M12 runs inside this container as sllidar_node. Stopping the
    # container therefore stops the camera, cuVSLAM, A2M12 LiDAR, and
    # slam_toolbox together; RViz is intentionally a separate observer.
    lidar_stop="not_running"
    if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
      service_output=""
      service_rc=0
      service_output="$(docker exec "${CONTAINER}" bash -lc \
        "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; timeout 5s ros2 service call /stop_motor std_srvs/srv/Empty '{}'" 2>&1)" || service_rc=$?
      printf '%s\n' "${service_output}"
      # ros2 service call can print a valid response and still return the
      # timeout code when its CLI does not exit at the deadline. Treat the
      # response as authoritative, while retaining force cleanup otherwise.
      if [[ "${service_output}" == *"response:"* && "${service_output}" == *"Empty_Response"* ]]; then
        lidar_stop="stop_motor_service_called"
      else
        lidar_stop="stop_motor_service_failed_force_cleanup"
      fi
    fi
    docker stop --time 10 "${CONTAINER}" 2>/dev/null || true
    docker rm -f "${CONTAINER}" 2>/dev/null || true
    echo "cleaned container=${CONTAINER} stopped_components=d435i,cuvslam,a2m12_lidar,slam_toolbox lidar_stop=${lidar_stop}"
    ;;
  status)
    docker ps --filter "name=^/${CONTAINER}$" --format '{{.Names}} {{.Status}}'
    ;;
  preflight)
    if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
      echo "RPLIDAR mapping container is not running; start it first with estimator=cuvslam" >&2
      exit 1
    fi
    docker exec "${CONTAINER}" bash \
      /workspaces/isaac_ros-dev/real_robot/cuvslam/tools/check_real_d435i_lidar_ekf_dependencies.sh
    ;;
  logs)
    docker logs "${CONTAINER}"
    ;;
  telemetry)
    docker exec "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec python3 /workspaces/isaac_ros-dev/real_robot/cuvslam/tools/collect_real_d435i_odom_live.py"
    ;;
  quality)
    docker exec "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec python3 /workspaces/isaac_ros-dev/real_robot/cuvslam/tools/collect_real_d435i_image_quality.py"
    ;;
  record)
    duration="${2:-30}"
    run_name="${3:?run name is required}"
    case "${run_name}" in
      [a-zA-Z0-9_.-]*) ;;
      *) echo "invalid run name" >&2; exit 2 ;;
    esac
    output="/workspaces/isaac_ros_data/experiments/real_d435i_rplidar/${run_name}/rosbag"
    docker exec "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; mkdir -p /workspaces/isaac_ros_data/experiments/real_d435i_rplidar/${run_name}; timeout --signal=INT ${duration}s ros2 bag record -o ${output} --compression-mode file --compression-format zstd ${light_record_topics[*]}"
    ;;
  record_start)
    run_name="${2:?run name is required}"
    record_mode="${3:-light}"
    case "${run_name}" in
      [a-zA-Z0-9_.-]*) ;;
      *) echo "invalid run name" >&2; exit 2 ;;
    esac
    case "${record_mode}" in
      light)
        selected_topics=("${light_record_topics[@]}")
        required_free_mb="${RPLIDAR_LIGHT_MIN_FREE_MB}"
        ;;
      full)
        selected_topics=("${full_record_topics[@]}")
        required_free_mb="${RPLIDAR_FULL_MIN_FREE_MB}"
        ;;
      *) echo "record mode must be light or full" >&2; exit 2 ;;
    esac
    if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
      echo "RPLIDAR mapping container is not running" >&2
      exit 1
    fi
    available_kb="$(df -Pk "${DATA_DIR}" | awk 'NR == 2 {print $4}')"
    available_mb="$((available_kb / 1024))"
    if (( available_mb < required_free_mb )); then
      echo "recording_preflight_failed reason=low_disk_space mode=${record_mode} available_mb=${available_mb} required_mb=${required_free_mb}" >&2
      exit 3
    fi
    output_dir="${RUN_DIR}/${run_name}"
    output="/workspaces/isaac_ros_data/experiments/real_d435i_rplidar/${run_name}/rosbag"
    container_output_dir="/workspaces/isaac_ros_data/experiments/real_d435i_rplidar/${run_name}"
    mkdir -p "${output_dir}"
    if docker exec "${CONTAINER}" bash -lc "test -s ${RPLIDAR_BAG_PID_FILE} && kill -0 \$(cat ${RPLIDAR_BAG_PID_FILE}) 2>/dev/null"; then
      echo "recording_already_running run=${run_name}"
      exit 0
    fi
    topic_args="${selected_topics[*]}"
    docker exec -d "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; mkdir -p ${container_output_dir}; ros2 bag record -o ${output} --compression-mode file --compression-format zstd ${topic_args} >${container_output_dir}/rosbag_record.log 2>&1 & bag_pid=\$!; echo \${bag_pid} >${RPLIDAR_BAG_PID_FILE}; while kill -0 \${bag_pid} 2>/dev/null; do available_kb=\$(df -Pk /workspaces/isaac_ros_data | awk 'NR == 2 {print \$4}'); if (( available_kb < ${RPLIDAR_RECORDING_RESERVE_MB} * 1024 )); then echo 'recording_stopped_by_disk_guard reserve_mb=${RPLIDAR_RECORDING_RESERVE_MB}' >>${container_output_dir}/rosbag_record.log; kill -INT \${bag_pid} 2>/dev/null || true; break; fi; sleep 2; done; wait \${bag_pid}; rm -f ${RPLIDAR_BAG_PID_FILE}"
    sleep 1
    if ! docker exec "${CONTAINER}" bash -lc "test -s ${RPLIDAR_BAG_PID_FILE} && kill -0 \$(cat ${RPLIDAR_BAG_PID_FILE}) 2>/dev/null"; then
      echo "recording_start_failed run=${run_name} mode=${record_mode}" >&2
      docker exec "${CONTAINER}" bash -lc "tail -20 ${container_output_dir}/rosbag_record.log 2>/dev/null" >&2 || true
      exit 1
    fi
    echo "recording_started run=${run_name} mode=${record_mode} compression=zstd available_mb=${available_mb} reserve_mb=${RPLIDAR_RECORDING_RESERVE_MB} dir=${output_dir}"
    ;;
  record_stop)
    run_name="${2:?run name is required}"
    case "${run_name}" in
      [a-zA-Z0-9_.-]*) ;;
      *) echo "invalid run name" >&2; exit 2 ;;
    esac
    if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
      docker exec "${CONTAINER}" bash -lc \
        "if test -s ${RPLIDAR_BAG_PID_FILE}; then bag_pid=\$(cat ${RPLIDAR_BAG_PID_FILE}); kill -INT \${bag_pid} 2>/dev/null || true; for i in \$(seq 1 20); do kill -0 \${bag_pid} 2>/dev/null || break; sleep 1; done; rm -f ${RPLIDAR_BAG_PID_FILE}; fi"
    fi
    echo "recording_stopped run=${run_name} dir=${RUN_DIR}/${run_name}"
    ;;
  *)
    echo "Usage: $0 {start [stereo|imu] [640x360x30|848x480x30] [lighting] [standard|features] [cuvslam|ekf_shadow]|stop|cleanup|status|preflight|logs|telemetry|quality|record [SECONDS] RUN_NAME|record_start RUN_NAME [light|full]|record_stop RUN_NAME}" >&2
    exit 2
    ;;
esac
