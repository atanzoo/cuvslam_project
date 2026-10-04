#!/usr/bin/env bash
set -euo pipefail

ENV_FILE="${ISAAC_ROS_ENV_FILE:-/home/tseng/isaac_ros.env}"
if [[ ! -f "${ENV_FILE}" ]]; then
  echo "Missing ${ENV_FILE}; copy isaac_ros.env.example and choose a reviewed image." >&2
  exit 2
fi

# shellcheck disable=SC1090
source "${ENV_FILE}"
: "${ISAAC_ROS_IMAGE:?ISAAC_ROS_IMAGE must be set in ${ENV_FILE}}"

WORKSPACE="${ISAAC_ROS_WORKSPACE:-/home/tseng/isaac_ros_ws}"
DATA_DIR="${ISAAC_ROS_DATA:-/home/tseng/isaac_ros_ws/simulation/cuvslam/data}"
DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-42}"

case "${1:-shell}" in
  shell) shift ;;
  preflight) shift; PREFLIGHT=1 ;;
  *) echo "Usage: $0 [shell|preflight]" >&2; exit 2 ;;
esac

mkdir -p "${WORKSPACE}/src" "${DATA_DIR}"

if ! docker info >/dev/null 2>&1; then
  echo "Docker is unavailable for this user. Do not use sudo inside the container." >&2
  echo "Grant Docker access to the user, reconnect SSH, then retry." >&2
  exit 3
fi

args=(
  --rm -it --init --privileged
  --name isaac_ros_isolated
  --network host
  --ipc host
  --runtime nvidia
  --env ROS_DOMAIN_ID="${DOMAIN_ID}"
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  --env NVIDIA_VISIBLE_DEVICES=all
  --env NVIDIA_DRIVER_CAPABILITIES=all
  --volume "${WORKSPACE}:/workspaces/isaac_ros-dev"
  --volume "${DATA_DIR}:/workspaces/isaac_ros_data"
  --volume /dev:/dev
  --volume /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h:/usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h:ro
  --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11:ro
  --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11:ro
  --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10:ro
  --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10:ro
  --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so:ro
  --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4:ro
  --volume /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1:ro
  --volume /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro
  --volume /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api:ro
  --volume /tmp/argus_socket:/tmp/argus_socket
  --entrypoint /bin/bash
  --workdir /workspaces/isaac_ros-dev
)

if [[ "${PREFLIGHT:-0}" == 1 ]]; then
  command_args=(-lc 'set -e; source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"; ros2 pkg prefix isaac_ros_visual_slam; command -v nvcc; nvcc --version | tail -1; ls -l /dev/bus/usb')
else
  command_args=(-lc 'source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec bash')
fi

exec docker run "${args[@]}" "${ISAAC_ROS_IMAGE}" "${command_args[@]}"
