#!/usr/bin/env bash
set -euo pipefail

CONTAINER="experimental_candidate_localization"
IMAGE="${ISAAC_ROS_IMAGE:-isaac_ros_dev-aarch64:ekf-shadow}"
WORKSPACE="/home/tseng/isaac_ros_ws"
DATA_DIR="/home/tseng/isaac_ros_data/real_robot/cuvslam"
MAP_YAML="${DATA_DIR}/maps/20260924_092630_current_cuvslam_a2m12/map.yaml"
LAUNCH_FILE="/workspaces/isaac_ros-dev/real_robot/cuvslam/experimental_localization/experimental_candidate_map_localization.launch.py"
CONTAINER_MAP_YAML="/workspaces/isaac_ros_data/real_robot/cuvslam/maps/20260924_092630_current_cuvslam_a2m12/map.yaml"

is_running() {
  docker ps --format '{{.Names}}' | grep -Fxq "$1"
}

case "${1:-status}" in
  start)
    for conflicting in real_d435i_rplidar real_d435i_r2 real_d435i_r1p5_axis_gui; do
      if is_running "${conflicting}"; then
        echo "conflicting runtime is active: ${conflicting}; stop it before experimental localization" >&2
        exit 3
      fi
    done
    if is_running "${CONTAINER}"; then
      echo "already_running container=${CONTAINER}"
      exit 0
    fi
    if [[ ! -f "${MAP_YAML}" ]]; then
      echo "candidate map is missing: ${MAP_YAML}" >&2
      exit 4
    fi
    docker run -d --rm --init \
      --name "${CONTAINER}" \
      --network host --ipc host \
      --env ROS_DOMAIN_ID=42 \
      --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      --volume "${WORKSPACE}:/workspaces/isaac_ros-dev" \
      --volume "${DATA_DIR}:/workspaces/isaac_ros_data" \
      --workdir /workspaces/isaac_ros-dev \
      --entrypoint /bin/bash "${IMAGE}" \
      -lc "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec ros2 launch '${LAUNCH_FILE}' map_yaml:='${CONTAINER_MAP_YAML}'"
    echo "started container=${CONTAINER} map=${MAP_YAML} owner=candidate_amcl"
    ;;
  stop)
    if is_running "${CONTAINER}"; then
      docker stop --time 10 "${CONTAINER}"
    else
      echo "not_running container=${CONTAINER}"
    fi
    ;;
  status)
    docker ps --filter "name=^/${CONTAINER}$" --format '{{.Names}} {{.Status}}'
    ;;
  preflight)
    if [[ ! -f "${MAP_YAML}" ]]; then
      echo "candidate map is missing: ${MAP_YAML}" >&2
      exit 4
    fi
    sha256sum "${MAP_YAML}"
    if is_running "${CONTAINER}"; then
      docker exec "${CONTAINER}" bash -lc \
        "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; python3 -B -c 'from pathlib import Path; p=Path(\"${LAUNCH_FILE}\"); compile(p.read_text(), str(p), \"exec\"); print(\"launch syntax ok\")'"
    else
      echo "container=${CONTAINER} is not running"
    fi
    ;;
  telemetry)
    if ! is_running "${CONTAINER}"; then
      echo "localization container is not running; start it first" >&2
      exit 1
    fi
    docker exec "${CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec python3 /workspaces/isaac_ros-dev/real_robot/cuvslam/tools/collect_real_d435i_odom_live.py"
    ;;
  *)
    echo "usage: $0 {start|stop|status|preflight|telemetry}" >&2
    exit 2
    ;;
esac
