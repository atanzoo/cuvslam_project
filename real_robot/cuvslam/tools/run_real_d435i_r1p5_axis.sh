#!/usr/bin/env bash
set -euo pipefail

CONTAINER="real_d435i_r1p5_axis_gui"
IMAGE="isaac_ros_dev-aarch64:fixed"
WORKSPACE="/home/tseng/isaac_ros_ws"
DATA_DIR="/home/tseng/isaac_ros_data/real_robot/cuvslam"

case "${1:-status}" in
  start)
    if docker ps --format '{{.Names}}' | grep -Fxq "experimental_candidate_localization"; then
      echo "experimental localization is active; stop it before starting R1.5" >&2
      exit 3
    fi
    if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
      echo "already_running"
      exit 0
    fi
    mkdir -p "${DATA_DIR}/experiments/real_d435i_r1p5_gui"
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
      -lc 'source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_d435i_real.launch.py enable_imu_fusion:=true enable_mapping:=false enable_debug_mode:=false'
    ;;
  stop)
    docker stop "${CONTAINER}" 2>/dev/null || true
    ;;
  status)
    docker ps --filter "name=^/${CONTAINER}$" --format '{{.Names}} {{.Status}}'
    ;;
  phase)
    label="${2:?phase label is required}"
    duration="${3:-10}"
    output_name="${4:?output filename is required}"
    case "${label}" in
      [a-zA-Z0-9_-]*) ;;
      *) echo "invalid phase label" >&2; exit 2 ;;
    esac
    case "${output_name}" in
      [a-zA-Z0-9_.-]*) ;;
      *) echo "invalid output filename" >&2; exit 2 ;;
    esac
    output="/workspaces/isaac_ros_data/experiments/real_d435i_r1p5_gui/${output_name}"
    docker exec "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; python3 /workspaces/isaac_ros-dev/real_robot/cuvslam/tools/collect_real_d435i_axis_phase.py --label ${label} --duration ${duration} --live-period 0.25 --output ${output}"
    ;;
  *)
    echo "Usage: $0 {start|stop|status|phase LABEL [DURATION] OUTPUT_JSON}" >&2
    exit 2
    ;;
esac
