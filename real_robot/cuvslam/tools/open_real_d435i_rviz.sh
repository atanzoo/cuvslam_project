#!/usr/bin/env bash
set -euo pipefail

JETSON_HOST="tseng@192.168.55.1"
JETSON_PORT="22"
CONTAINER="real_d435i_rviz"
IMAGE="isaac_ros_dev-aarch64:fixed"
REMOTE_WORKSPACE="/home/tseng/isaac_ros_ws"
RVIZ_CONFIG="/workspaces/isaac_ros-dev/config/real_d435i_mapping.rviz"

exec ssh -p "${JETSON_PORT}" -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  -o ControlMaster=no -o ControlPath=none -o StrictHostKeyChecking=accept-new \
  "${JETSON_HOST}" \
  "docker rm -f ${CONTAINER} >/dev/null 2>&1 || true; \
   test -S /tmp/.X11-unix/X0; \
   test -f /run/user/124/gdm/Xauthority; \
   exec docker run --rm --name ${CONTAINER} --network host --ipc host --runtime nvidia \
     --env DISPLAY=:0 --env XAUTHORITY=/run/user/124/gdm/Xauthority \
     --env QT_X11_NO_MITSHM=1 --env XDG_RUNTIME_DIR=/tmp/runtime-root \
     --env ROS_DOMAIN_ID=42 \
     --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
     --env NVIDIA_VISIBLE_DEVICES=all --env NVIDIA_DRIVER_CAPABILITIES=all \
     --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
     --volume /run/user/124/gdm:/run/user/124/gdm:ro \
     --volume ${REMOTE_WORKSPACE}:/workspaces/isaac_ros-dev:ro \
     --workdir /workspaces/isaac_ros-dev --entrypoint /bin/bash ${IMAGE} \
     -lc 'mkdir -p /tmp/runtime-root; source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec rviz2 -d ${RVIZ_CONFIG}'"
