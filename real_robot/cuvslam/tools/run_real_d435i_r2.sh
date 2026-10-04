#!/usr/bin/env bash
set -euo pipefail

CONTAINER="real_d435i_r2"
IMAGE="isaac_ros_dev-aarch64:fixed"
WORKSPACE="/home/tseng/isaac_ros_ws"
DATA_DIR="/home/tseng/isaac_ros_data/real_robot/cuvslam"
R2_DIR="${DATA_DIR}/experiments/real_d435i_r2"
R2_BAG_PID_FILE="/tmp/real_d435i_r2_bag.pid"
R2_LIGHT_MIN_FREE_MB=512
R2_FULL_MIN_FREE_MB=4096
R2_RECORDING_RESERVE_MB=2048

light_record_topics=(
  /camera/infra1/camera_info
  /camera/infra1/metadata
  /camera/infra2/camera_info
  /camera/infra2/metadata
  /camera/imu
  /tf
  /tf_static
  /visual_slam/status
  /visual_slam/tracking/odometry
  /visual_slam/tracking/vo_pose
  /visual_slam/tracking/slam_path
  /visual_slam/vis/observations_cloud
  /visual_slam/vis/landmarks_cloud
  /visual_slam/vis/pose_graph_nodes
  /visual_slam/vis/pose_graph_edges
  /visual_slam/vis/loop_closure_cloud
)

full_record_topics=(
  /camera/infra1/image_rect_raw
  /camera/infra1/camera_info
  /camera/infra1/metadata
  /camera/infra2/image_rect_raw
  /camera/infra2/camera_info
  /camera/infra2/metadata
  /camera/imu
  /tf
  /tf_static
  /visual_slam/status
  /visual_slam/tracking/odometry
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
    if docker ps --format '{{.Names}}' | grep -Fxq "experimental_candidate_localization"; then
      echo "experimental localization is active; stop it before starting R2" >&2
      exit 3
    fi
    mode="${2:-stereo}"
    mapping_mode="${3:-mapping}"
    reset_mode="${4:-no_reset}"
    stack_mode="${5:-slam}"
    profile="${6:-640x360x30}"
    lighting="${7:-official}"
    diagnostic_mode="${8:-standard}"
    emitter_override="${9:-profile}"
    case "${mode}" in
      stereo) fusion=false; mode_name=stereo_only ;;
      imu) fusion=true; mode_name=stereo_imu ;;
      *) echo "mode must be stereo or imu" >&2; exit 2 ;;
    esac
    case "${mapping_mode}" in
      odom) mapping=false; slam_visualization=false; observations_view=false ;;
      mapping) mapping=true; slam_visualization=true; observations_view=false ;;
      *) echo "mapping mode must be odom or mapping" >&2; exit 2 ;;
    esac
    case "${reset_mode}" in
      reset) initial_reset=true ;;
      no_reset) initial_reset=false ;;
      *) echo "reset mode must be reset or no_reset" >&2; exit 2 ;;
    esac
    case "${stack_mode}" in
      slam) enable_visual_slam=true ;;
      camera) enable_visual_slam=false; mapping=false ;;
      *) echo "stack mode must be slam or camera" >&2; exit 2 ;;
    esac
    case "${profile}" in
      640x360x30) driver_profile="640,360,30" ;;
      848x480x30) driver_profile="848,480,30" ;;
      *) echo "profile must be 640x360x30 or 848x480x30" >&2; exit 2 ;;
    esac
    case "${lighting}" in
      official|low_light) denoise=true; emitter=1; auto_exposure=true; exposure=8500; gain=16 ;;
      low_light_motion)
        # Experimental motion profile: manual exposure/gain values require
        # brightness and tracking validation on the target D435i.
        denoise=true; emitter=1; auto_exposure=false; exposure=10000; gain=24 ;;
      baseline) denoise=false; emitter=1; auto_exposure=true; exposure=8500; gain=16 ;;
      robust) denoise=true; emitter=0; auto_exposure=true; exposure=8500; gain=16 ;;
      high_contrast) denoise=true; emitter=0; auto_exposure=false; exposure=8000; gain=16 ;;
      *) echo "lighting must be official, baseline, robust, low_light, low_light_motion, or high_contrast" >&2; exit 2 ;;
    esac
    case "${emitter_override}" in
      profile) ;;
      on) emitter=1 ;;
      off) emitter=0 ;;
      *) echo "emitter override must be profile, on, or off" >&2; exit 2 ;;
    esac
    case "${diagnostic_mode}" in
      standard) debug_mode=false ;;
      features) debug_mode=false; observations_view=true ;;
      *) echo "diagnostic mode must be standard or features" >&2; exit 2 ;;
    esac
    if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
      echo "already_running mode=${mode_name}"
      exit 0
    fi
    mkdir -p "${R2_DIR}"
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
      -lc "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_d435i_real.launch.py enable_imu_fusion:=${fusion} enable_mapping:=${mapping} enable_debug_mode:=${debug_mode} enable_slam_visualization:=${slam_visualization} enable_observations_view:=${observations_view} initial_reset:=${initial_reset} enable_visual_slam:=${enable_visual_slam} infra_profile:=${driver_profile} denoise_input_images:=${denoise} emitter_enabled:=${emitter} enable_auto_exposure:=${auto_exposure} exposure:=${exposure} gain:=${gain} debug_dump_path:=/workspaces/isaac_ros_data/experiments/real_d435i_r2/debug_dump"
    echo "started mode=${mode_name} mapping=${mapping_mode} reset=${reset_mode} stack=${stack_mode} profile=${profile} lighting=${lighting} emitter_override=${emitter_override} diagnostic=${diagnostic_mode} denoise=${denoise} emitter=${emitter} auto_exposure=${auto_exposure} exposure=${exposure} gain=${gain} slam_visualization=${slam_visualization} observations_view=${observations_view} container=${CONTAINER}"
    ;;
  stop|cleanup)
    # The R2 container is launched with --rm, but explicitly remove it as a
    # second step so a failed launch or interrupted docker stop cannot leak a
    # stale container into the next experiment.
    if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
      docker stop --time 10 "${CONTAINER}" 2>/dev/null || true
    fi
    if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
      docker rm -f "${CONTAINER}" 2>/dev/null || true
    fi
    echo "cleaned container=${CONTAINER}"
    ;;
  status)
    docker ps --filter "name=^/${CONTAINER}$" --format '{{.Names}} {{.Status}}'
    ;;
  collect)
    duration="${2:-30}"
    output_name="${3:?output filename is required}"
    mode_name="${4:-stereo_only}"
    case "${output_name}" in [a-zA-Z0-9_.-]*) ;; *) echo "invalid output filename" >&2; exit 2 ;; esac
    output="/workspaces/isaac_ros_data/experiments/real_d435i_r2/${output_name}"
    docker exec "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; python3 /workspaces/isaac_ros-dev/real_robot/cuvslam/tools/collect_real_d435i_r2_run.py --duration ${duration} --mode ${mode_name} --output ${output}"
    ;;
  record)
    duration="${2:-30}"
    run_name="${3:?run name is required}"
    case "${run_name}" in [a-zA-Z0-9_.-]*) ;; *) echo "invalid run name" >&2; exit 2 ;; esac
    output="/workspaces/isaac_ros_data/experiments/real_d435i_r2/${run_name}/rosbag"
    docker exec "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; mkdir -p /workspaces/isaac_ros_data/experiments/real_d435i_r2/${run_name}; timeout --signal=INT ${duration}s ros2 bag record -o ${output} /camera/infra1/image_rect_raw /camera/infra1/camera_info /camera/infra2/image_rect_raw /camera/infra2/camera_info /camera/imu /tf /tf_static /visual_slam/status /visual_slam/tracking/odometry /visual_slam/tracking/vo_pose /visual_slam/tracking/slam_path /visual_slam/vis/landmarks_cloud"
    ;;
  record_start)
    run_name="${2:?run name is required}"
    record_mode="${3:-light}"
    case "${run_name}" in [a-zA-Z0-9_.-]*) ;; *) echo "invalid run name" >&2; exit 2 ;; esac
    case "${record_mode}" in
      light)
        selected_topics=("${light_record_topics[@]}")
        required_free_mb="${R2_LIGHT_MIN_FREE_MB}"
        ;;
      full)
        selected_topics=("${full_record_topics[@]}")
        required_free_mb="${R2_FULL_MIN_FREE_MB}"
        ;;
      *) echo "record mode must be light or full" >&2; exit 2 ;;
    esac
    compression_args=(--compression-mode file --compression-format zstd)
    output_dir="${R2_DIR}/${run_name}"
    output="/workspaces/isaac_ros_data/experiments/real_d435i_r2/${run_name}/rosbag"
    if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
      echo "R2 container is not running" >&2
      exit 1
    fi
    available_kb="$(df -Pk "${DATA_DIR}" | awk 'NR == 2 {print $4}')"
    available_mb="$((available_kb / 1024))"
    if (( available_mb < required_free_mb )); then
      echo "recording_preflight_failed reason=low_disk_space mode=${record_mode} available_mb=${available_mb} required_mb=${required_free_mb}" >&2
      exit 3
    fi
    mkdir -p "${output_dir}"
    if docker exec "${CONTAINER}" bash -lc "test -s ${R2_BAG_PID_FILE} && kill -0 \$(cat ${R2_BAG_PID_FILE}) 2>/dev/null"; then
      echo "recording_already_running run=${run_name}"
      exit 0
    fi
    topic_args="${selected_topics[*]}"
    compression_args_text="${compression_args[*]}"
    container_output_dir="/workspaces/isaac_ros_data/experiments/real_d435i_r2/${run_name}"
    docker exec -d "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; mkdir -p ${container_output_dir}; ros2 bag record -o ${output} ${compression_args_text} ${topic_args} >${container_output_dir}/rosbag_record.log 2>&1 & bag_pid=\$!; echo \${bag_pid} >${R2_BAG_PID_FILE}; while kill -0 \${bag_pid} 2>/dev/null; do available_kb=\$(df -Pk /workspaces/isaac_ros_data | awk 'NR == 2 {print \$4}'); if (( available_kb < ${R2_RECORDING_RESERVE_MB} * 1024 )); then echo 'recording_stopped_by_disk_guard reserve_mb=${R2_RECORDING_RESERVE_MB}' >>${container_output_dir}/rosbag_record.log; kill -INT \${bag_pid} 2>/dev/null || true; break; fi; sleep 2; done; wait \${bag_pid}; rm -f ${R2_BAG_PID_FILE}"
    sleep 1
    if ! docker exec "${CONTAINER}" bash -lc "test -s ${R2_BAG_PID_FILE} && kill -0 \$(cat ${R2_BAG_PID_FILE}) 2>/dev/null"; then
      echo "recording_start_failed run=${run_name} mode=${record_mode}" >&2
      docker exec "${CONTAINER}" bash -lc "tail -20 ${container_output_dir}/rosbag_record.log 2>/dev/null" >&2 || true
      exit 1
    fi
    echo "recording_started run=${run_name} mode=${record_mode} compression=zstd available_mb=${available_mb} reserve_mb=${R2_RECORDING_RESERVE_MB} dir=${output_dir}"
    ;;
  record_stop)
    run_name="${2:?run name is required}"
    case "${run_name}" in [a-zA-Z0-9_.-]*) ;; *) echo "invalid run name" >&2; exit 2 ;; esac
    if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
      docker exec "${CONTAINER}" bash -lc \
        "if test -s ${R2_BAG_PID_FILE}; then bag_pid=\$(cat ${R2_BAG_PID_FILE}); kill -INT \${bag_pid} 2>/dev/null || true; for i in \$(seq 1 20); do kill -0 \${bag_pid} 2>/dev/null || break; sleep 1; done; rm -f ${R2_BAG_PID_FILE}; fi"
    fi
    echo "recording_stopped run=${run_name} dir=${R2_DIR}/${run_name}"
    ;;
  run)
    duration="${2:-30}"
    run_name="${3:?run name is required}"
    mode_name="${4:-stereo_only}"
    route="${5:-r2_manual_short_straight}"
    case "${run_name}" in [a-zA-Z0-9_.-]*) ;; *) echo "invalid run name" >&2; exit 2 ;; esac
    case "${mode_name}" in stereo_only|stereo_imu) ;; *) echo "invalid mode name" >&2; exit 2 ;; esac
    output_dir="/workspaces/isaac_ros_data/experiments/real_d435i_r2/${run_name}"
    bag_output="${output_dir}/rosbag"
    json_output="${output_dir}/odometry.json"
    docker exec "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; mkdir -p ${output_dir}; set +e; ros2 bag record -o ${bag_output} /camera/infra1/image_rect_raw /camera/infra1/camera_info /camera/infra1/metadata /camera/infra2/image_rect_raw /camera/infra2/camera_info /camera/infra2/metadata /camera/imu /tf /tf_static /visual_slam/status /visual_slam/tracking/odometry /visual_slam/tracking/vo_pose /visual_slam/tracking/slam_path /visual_slam/vis/observations_cloud /visual_slam/vis/landmarks_cloud /visual_slam/vis/pose_graph_nodes /visual_slam/vis/pose_graph_edges /visual_slam/vis/loop_closure_cloud >${output_dir}/rosbag_record.log 2>&1 & bag_pid=\$!; sleep 2; python3 /workspaces/isaac_ros-dev/real_robot/cuvslam/tools/collect_real_d435i_r2_run.py --duration ${duration} --mode ${mode_name} --route ${route} --output ${json_output}; collector_rc=\$?; kill -INT \$bag_pid 2>/dev/null || true; wait \$bag_pid 2>/dev/null || true; exit \$collector_rc"
    ;;
  telemetry)
    docker exec "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec python3 /workspaces/isaac_ros-dev/real_robot/cuvslam/tools/collect_real_d435i_odom_live.py"
    ;;
  quality)
    docker exec "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec python3 /workspaces/isaac_ros-dev/real_robot/cuvslam/tools/collect_real_d435i_image_quality.py"
    ;;
  *)
    echo "Usage: $0 {start [stereo|imu] [odom|mapping] [reset|no_reset] [slam|camera] [profile] [lighting] [standard|features]|stop|cleanup|status|collect [SECONDS] OUTPUT_JSON [MODE]|record [SECONDS] RUN_NAME|run [SECONDS] RUN_NAME [MODE] [ROUTE]|telemetry|quality}" >&2
    exit 2
    ;;
esac
