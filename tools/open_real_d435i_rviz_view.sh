#!/usr/bin/env bash
set -euo pipefail

# Idempotent Jetson-side helper for the Mac R2 GUI.
# It reuses the GPU-backed RViz container when possible and exposes only the
# RViz window through localhost VNC. The Mac connects through an SSH tunnel.

CONTAINER="real_d435i_rviz"
IMAGE="isaac_ros_dev-aarch64:rviz-vnc"
REMOTE_WORKSPACE="/home/tseng/isaac_ros_ws"
RVIZ_CONFIG="/workspaces/isaac_ros-dev/config/real_d435i_mapping.rviz"
DISPLAY_NUM=":0"
XAUTHORITY_FILE="/run/user/124/gdm/Xauthority"
VNC_PORT="5900"
BRIDGE_BINARY="/tmp/rviz_x11_bridge"
BRIDGE_SOURCE="/workspaces/isaac_ros-dev/tools/rviz_x11_bridge.c"
BRIDGE_ID_FILE="/tmp/rviz_x11_bridge.window"
BRIDGE_LOG="/tmp/rviz_x11_bridge.log"

# Keep the VNC credential outside the repository. On the Jetson, create
# ${REMOTE_WORKSPACE}/.local.env (ignored by Git) or export the variable in
# the calling environment:
#   CUVSLAM_VNC_PASSWORD='...'
REMOTE_ENV_FILE="${CUVSLAM_LOCAL_ENV_FILE:-${REMOTE_WORKSPACE}/.local.env}"
if [[ -f "${REMOTE_ENV_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${REMOTE_ENV_FILE}"
  set +a
fi
VNC_PASSWORD="${CUVSLAM_VNC_PASSWORD:-}"

container_running() {
  docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"
}

start_rviz_container() {
  docker run -d --rm --name "${CONTAINER}" --network host --ipc host --runtime nvidia \
    --env DISPLAY="${DISPLAY_NUM}" \
    --env XAUTHORITY="${XAUTHORITY_FILE}" \
    --env QT_X11_NO_MITSHM=1 \
    --env XDG_RUNTIME_DIR=/tmp/runtime-root \
    --env ROS_DOMAIN_ID=42 \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --env NVIDIA_VISIBLE_DEVICES=all \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume /run/user/124/gdm:/run/user/124/gdm:ro \
    --volume "${REMOTE_WORKSPACE}:/workspaces/isaac_ros-dev:ro" \
    --workdir /workspaces/isaac_ros-dev \
    --entrypoint /bin/bash "${IMAGE}" \
    -lc "mkdir -p /tmp/runtime-root; source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec rviz2 -d ${RVIZ_CONFIG}" \
    >/dev/null
}

rviz_window_id() {
  docker exec "${CONTAINER}" bash -lc \
    'xwininfo -root -tree 2>/dev/null | awk "/real_d435i_mapping.rviz/ {print \$1; exit}"' \
    2>/dev/null || true
}

wait_for_window() {
  local window_id=""
  for _ in $(seq 1 30); do
    window_id="$(rviz_window_id)"
    if [[ "${window_id}" =~ ^0x[0-9a-fA-F]+$ ]]; then
      printf '%s\n' "${window_id}"
      return 0
    fi
    sleep 1
  done
  return 1
}

start_vnc() {
  local bridge_id="$1"
  local password_arg=""
  if x11vnc_for_bridge_alive "${bridge_id}"; then
    return 0
  fi
  printf -v password_arg '%q' "${VNC_PASSWORD}"
  docker exec "${CONTAINER}" bash -lc 'pkill -x x11vnc 2>/dev/null || true'
  docker exec -d "${CONTAINER}" bash -lc \
    "x11vnc -display ${DISPLAY_NUM} -id ${bridge_id} -rfbport ${VNC_PORT} -localhost -forever -shared -passwd ${password_arg} -noxdamage -noxcomposite -noshm -noscr -nowf -noxfixes -noxrecord -allinput -input_eagerly -always_inject -input KMBCF -o /tmp/x11vnc.log -logappend /tmp/x11vnc.log" \
    >/dev/null
  for _ in $(seq 1 10); do
    if x11vnc_for_bridge_alive "${bridge_id}"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

bridge_alive() {
  docker exec "${CONTAINER}" bash -lc \
    'for pid in $(pgrep -x rviz_x11_bridge 2>/dev/null); do state=$(awk "{print \$3}" /proc/${pid}/stat 2>/dev/null || true); [[ "${state}" != "Z" ]] && exit 0; done; exit 1'
}

bridge_window_id() {
  docker exec "${CONTAINER}" bash -lc \
    "id=\$(cat ${BRIDGE_ID_FILE} 2>/dev/null || true); if [[ \"\${id}\" =~ ^0x[0-9a-fA-F]+$ ]] && xwininfo -id \"\${id}\" >/dev/null 2>&1; then printf '%s\\n' \"\${id}\"; fi" \
    2>/dev/null || true
}

start_bridge() {
  local target_id="$1"
  local bridge_id=""
  if bridge_alive; then
    bridge_id="$(bridge_window_id)"
    if [[ -n "${bridge_id}" ]]; then
      printf '%s\n' "${bridge_id}"
      return 0
    fi
  fi

  docker exec "${CONTAINER}" bash -lc \
    "pkill -x x11vnc 2>/dev/null || true; pkill -x rviz_x11_bridge 2>/dev/null || true; gcc -O2 -Wall -Wextra -o ${BRIDGE_BINARY} ${BRIDGE_SOURCE} -lX11 -lXext -lXtst; rm -f ${BRIDGE_ID_FILE} ${BRIDGE_LOG}" \
    >/dev/null
  docker exec -d "${CONTAINER}" bash -lc \
    "nohup ${BRIDGE_BINARY} ${target_id} ${BRIDGE_ID_FILE} >${BRIDGE_LOG} 2>&1 </dev/null &" \
    >/dev/null

  for _ in $(seq 1 20); do
    bridge_id="$(bridge_window_id)"
    if [[ -n "${bridge_id}" ]] && bridge_alive; then
      printf '%s\n' "${bridge_id}"
      return 0
    fi
    sleep 1
  done
  docker exec "${CONTAINER}" bash -lc "tail -n 40 ${BRIDGE_LOG} 2>/dev/null || true" >&2
  return 1
}

x11vnc_for_bridge_alive() {
  local bridge_id="$1"
  docker exec "${CONTAINER}" bash -lc \
    "for pid in \$(pgrep -x x11vnc 2>/dev/null); do state=\$(awk \"{print \\\$3}\" /proc/\${pid}/stat 2>/dev/null || true); args=\$(tr \\\0 ' ' </proc/\${pid}/cmdline 2>/dev/null || true); [[ \"\${state}\" != \"Z\" && \"\${args}\" == *\" -id ${bridge_id}\"* ]] && exit 0; done; exit 1" \
    2>/dev/null
}

set_virtual_screen() {
  # The headless GDM X session starts at 640x480. Keep the RViz canvas large
  # across reconnects/reboots without touching the cuVSLAM container.
  docker exec "${CONTAINER}" bash -lc \
    "DISPLAY=${DISPLAY_NUM} XAUTHORITY=${XAUTHORITY_FILE} xrandr --fb 640x480 >/dev/null 2>&1 || true"
}

case "${1:-open}" in
  open)
    if [[ -z "${VNC_PASSWORD}" ]]; then
      echo "CUVSLAM_VNC_PASSWORD is not configured; set it in ${REMOTE_ENV_FILE} (not in Git)" >&2
      exit 2
    fi
    test -S /tmp/.X11-unix/X0
    if ! container_running; then
      start_rviz_container
    fi
    set_virtual_screen
    window_id="$(wait_for_window)" || {
      echo "RViz window was not found" >&2
      docker logs --tail 40 "${CONTAINER}" >&2 || true
      exit 1
    }
    bridge_id="$(start_bridge "${window_id}")" || {
      echo "RViz display bridge did not start" >&2
      exit 1
    }
    start_vnc "${bridge_id}" || {
      echo "x11vnc did not start" >&2
      docker exec "${CONTAINER}" bash -lc 'ps -eo pid,stat,args | grep -E "(rviz_x11_bridge|x11vnc)" | grep -v grep || true' >&2
      exit 1
    }
    echo "rviz_ready window=${window_id} bridge=${bridge_id} vnc=127.0.0.1:${VNC_PORT} input=enabled password=configured"
    ;;
  reset)
    # RViz keeps the last received landmarks/path in its displays even after
    # the R2 publisher exits. Restart the dedicated RViz container to start
    # the next experiment with an empty visual scene.
    if container_running; then
      docker stop --time 10 "${CONTAINER}" >/dev/null 2>&1 || true
    fi
    exec "$0" open
    ;;
  status)
    if ! container_running; then
      echo "rviz_container=stopped"
      exit 1
    fi
    echo "rviz_container=running window=$(rviz_window_id)"
    docker exec "${CONTAINER}" bash -lc 'ps -eo pid,stat,args | grep -E "(rviz_x11_bridge|x11vnc)" | grep -v grep || true'
    ;;
  *)
    echo "Usage: $0 {open|reset|status}" >&2
    exit 2
    ;;
esac
