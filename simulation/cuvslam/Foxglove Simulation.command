#!/bin/bash
set -euo pipefail

JETSON_USER="tseng"
JETSON_HOSTS=("192.168.55.1" "192.168.2.2")
JETSON_SSH_PORT="22"
ISAAC_CONTAINER="isaac_ros_dev-aarch64-container"
REMOTE_BRIDGE_PORT="8765"
LOCAL_TUNNEL_PORT="8766"
BRIDGE_LAUNCH="foxglove_bridge_launch.xml"
BRIDGE_LOG="/tmp/cuvslam-foxglove-bridge.log"
FOXGLOVE_APP="/Applications/Foxglove.app"
FOXGLOVE_URL="foxglove://open?ds=foxglove-websocket&ds.url=ws%3A%2F%2F127.0.0.1%3A${LOCAL_TUNNEL_PORT}"

SSH_OPTIONS=(
  -p "${JETSON_SSH_PORT}"
  -o BatchMode=yes
  -o ConnectTimeout=5
  -o ServerAliveInterval=15
  -o ServerAliveCountMax=3
  -o TCPKeepAlive=yes
  -o ControlMaster=no
  -o ControlPath=none
  -o StrictHostKeyChecking=accept-new
)

TUNNEL_PID=""
BRIDGE_STARTED=0
JETSON_TARGET=""

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${TUNNEL_PID}" ]] && kill -0 "${TUNNEL_PID}" 2>/dev/null; then
    kill "${TUNNEL_PID}" 2>/dev/null || true
    wait "${TUNNEL_PID}" 2>/dev/null || true
  fi
  if [[ "${BRIDGE_STARTED}" -eq 1 ]] && [[ -n "${JETSON_TARGET}" ]]; then
    echo "Stopping the Foxglove bridge started by this launcher ..."
    ssh "${SSH_OPTIONS[@]}" "${JETSON_TARGET}" \
      docker exec "${ISAAC_CONTAINER}" \
      pkill -INT -f "[f]oxglove_bridge_launch.xml" >/dev/null 2>&1 || true
    sleep 1
    ssh "${SSH_OPTIONS[@]}" "${JETSON_TARGET}" \
      docker exec "${ISAAC_CONTAINER}" \
      pkill -TERM -f "[f]oxglove_bridge_launch.xml" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if [[ ! -d "${FOXGLOVE_APP}" ]]; then
  echo "Foxglove is not installed at ${FOXGLOVE_APP}."
  echo "Install the Foxglove desktop app, then run this launcher again."
  read -r -p "Press Return to close."
  exit 1
fi

if /usr/sbin/lsof -nP -iTCP:"${LOCAL_TUNNEL_PORT}" -sTCP:LISTEN >/dev/null 2>&1; then
  echo "Local port ${LOCAL_TUNNEL_PORT} is already in use."
  echo "Close the previous Foxglove Simulation terminal before starting another."
  read -r -p "Press Return to close."
  exit 1
fi

for candidate in "${JETSON_HOSTS[@]}"; do
  target="${JETSON_USER}@${candidate}"
  echo "Checking ${target}:${JETSON_SSH_PORT} ..."
  if ssh "${SSH_OPTIONS[@]}" "${target}" true >/dev/null 2>&1; then
    JETSON_TARGET="${target}"
    break
  fi
done

if [[ -z "${JETSON_TARGET}" ]]; then
  echo "No configured Jetson SSH endpoint is reachable."
  read -r -p "Reconnect the Jetson, then press Return to close."
  exit 1
fi
echo "Using ${JETSON_TARGET}."

existing_bridge="$(
  ssh "${SSH_OPTIONS[@]}" "${JETSON_TARGET}" \
    docker exec "${ISAAC_CONTAINER}" \
    pgrep -af "[f]oxglove_bridge_launch.xml" 2>/dev/null || true
)"
if [[ -n "${existing_bridge}" ]]; then
  if [[ "${existing_bridge}" != *"address:=127.0.0.1"* ]] ||
     [[ "${existing_bridge}" != *"capabilities:=[connectionGraph]"* ]] ||
     [[ "${existing_bridge}" != *"simulation/.*"* ]]; then
    echo "A Foxglove bridge is already running with different settings:"
    echo "${existing_bridge}"
    echo "Stop or review that bridge before using this read-only launcher."
    read -r -p "Press Return to close."
    exit 1
  fi
  echo "Reusing the existing project read-only Foxglove bridge."
else
  bridge_shell="
set -e
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=43
export ROS2CLI_DISABLE_DAEMON=1
exec ros2 launch foxglove_bridge ${BRIDGE_LAUNCH} \
  port:=${REMOTE_BRIDGE_PORT} \
  address:=127.0.0.1 \
  \"topic_whitelist:=['^/(clock|tf|tf_static|scan|ground_truth/odom|d435i/.*|visual_slam/.*|simulation/.*)\$']\" \
  \"capabilities:=[connectionGraph]\" \
  \"client_topic_whitelist:=['^\$']\" \
  \"service_whitelist:=['^\$']\" \
  \"param_whitelist:=['^\$']\" \
  \"asset_uri_allowlist:=['^\$']\" \
  include_hidden:=false > ${BRIDGE_LOG} 2>&1
"
  printf -v remote_start_command '%q ' \
    docker exec -d "${ISAAC_CONTAINER}" /bin/bash -lc "${bridge_shell}"
  ssh "${SSH_OPTIONS[@]}" "${JETSON_TARGET}" "${remote_start_command}"
  BRIDGE_STARTED=1
  echo "Started the read-only Foxglove bridge in ROS domain 43."
fi

bridge_ready=0
for _ in {1..15}; do
  if ssh "${SSH_OPTIONS[@]}" "${JETSON_TARGET}" ss -ltn 2>/dev/null |
     /usr/bin/grep -q "127.0.0.1:${REMOTE_BRIDGE_PORT}"; then
    bridge_ready=1
    break
  fi
  sleep 1
done
if [[ "${bridge_ready}" -ne 1 ]]; then
  echo "Foxglove bridge did not open 127.0.0.1:${REMOTE_BRIDGE_PORT}."
  ssh "${SSH_OPTIONS[@]}" "${JETSON_TARGET}" \
    docker exec "${ISAAC_CONTAINER}" tail -n 30 "${BRIDGE_LOG}" 2>/dev/null || true
  read -r -p "Press Return to close."
  exit 1
fi

echo "Opening SSH tunnel on 127.0.0.1:${LOCAL_TUNNEL_PORT} ..."
ssh "${SSH_OPTIONS[@]}" \
  -N \
  -L "127.0.0.1:${LOCAL_TUNNEL_PORT}:127.0.0.1:${REMOTE_BRIDGE_PORT}" \
  -o ExitOnForwardFailure=yes \
  "${JETSON_TARGET}" &
TUNNEL_PID=$!

tunnel_ready=0
for _ in {1..10}; do
  if /usr/bin/nc -z 127.0.0.1 "${LOCAL_TUNNEL_PORT}" >/dev/null 2>&1; then
    tunnel_ready=1
    break
  fi
  if ! kill -0 "${TUNNEL_PID}" 2>/dev/null; then
    break
  fi
  sleep 0.5
done
if [[ "${tunnel_ready}" -ne 1 ]]; then
  echo "SSH tunnel did not become ready."
  read -r -p "Press Return to close."
  exit 1
fi

open "${FOXGLOVE_URL}"
echo
echo "Foxglove connection ready: ws://127.0.0.1:${LOCAL_TUNNEL_PORT}"
echo "This connection is read-only and limited to simulation/cuvSLAM topics."
echo "Keep this Terminal window open. Press Control-C to disconnect."
wait "${TUNNEL_PID}"
