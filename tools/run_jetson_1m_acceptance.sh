#!/usr/bin/env bash
set -Eeuo pipefail

container=isaac_ros_dev-aarch64-container
gazebo_unit=cuvslam-final-1m-gazebo.service
bridge_unit=cuvslam-final-1m-bridge.service
test_label="${CUVSLAM_TEST_LABEL:-acceptance_1m}"
partition="${CUVSLAM_TEST_PARTITION:-cuvslam_final_1m_20260731}"
world="${CUVSLAM_TEST_WORLD:-/home/tseng/jetson_slam_ws/install/slam_gazebo/share/slam_gazebo/worlds/cuvslam_mapping_simple.sdf}"
run_id="${test_label}_20260731_$$"
host_output="/home/tseng/isaac_ros_ws/data/${run_id}"
container_output="/workspaces/isaac_ros-dev/data/${run_id}"

publish_zero() {
  if docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null |
      grep -qx true; then
    docker exec "${container}" bash -lc \
      'source /opt/ros/humble/setup.bash;
       source /workspaces/isaac_ros-dev/install/setup.bash;
       export ROS_DOMAIN_ID=43;
       timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
       "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true' \
      || true
  fi
}

stop_vslam() {
  if docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null |
      grep -qx true; then
    vslam_pid="$(
      docker exec "${container}" pgrep -f \
        '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch isaac_ros_visual_slam' \
        2>/dev/null | head -n 1
    )"
    if [[ -n "${vslam_pid}" ]]; then
      docker exec "${container}" kill -INT "${vslam_pid}" || true
      sleep 5
    fi
  fi
}

cleanup() {
  set +e
  publish_zero
  stop_vslam
  systemctl --user stop "${bridge_unit}" "${gazebo_unit}"
  sleep 4
  docker stop "${container}" >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

if ps -eo cmd | grep -E \
    '[i]gn gazebo|[p]arameter_bridge|[r]os2 bag record|[s]imulation_truth_distance_profile' \
    >/dev/null; then
  echo "another simulation or recorder process is active" >&2
  exit 3
fi

docker start "${container}" >/dev/null
mkdir -p "${host_output}/debug_dump"

systemd-run --user \
  --unit="${gazebo_unit%.service}" \
  --collect \
  --property=KillMode=control-group \
  --property=TimeoutStopSec=8s \
  /bin/bash -lc \
  "source /opt/ros/foxy/setup.bash;
   source /home/tseng/jetson_slam_ws/install/setup.bash;
   export ROS_DOMAIN_ID=43;
   exec ros2 launch slam_gazebo gz_sim.launch.py world:=${world} ign_partition:=${partition} use_gui:=false"

systemd-run --user \
  --unit="${bridge_unit%.service}" \
  --collect \
  --property=KillMode=control-group \
  --property=TimeoutStopSec=8s \
  /bin/bash -lc \
  "source /opt/ros/foxy/setup.bash;
   source /home/tseng/jetson_slam_ws/install/setup.bash;
   export ROS_DOMAIN_ID=43;
   exec ros2 launch slam_gazebo gz_bridge_portable.launch.py ign_partition:=${partition}"

for _ in $(seq 1 30); do
  topics="$(
    bash -lc \
      'source /opt/ros/foxy/setup.bash;
       export ROS_DOMAIN_ID=43;
       export ROS2CLI_DISABLE_DAEMON=1;
       timeout 4 ros2 topic list' 2>/dev/null || true
  )"
  if grep -Fqx /clock <<<"${topics}" &&
      grep -Fqx /d435i/infra1/image_rect_raw <<<"${topics}" &&
      grep -Fqx /ground_truth/odom <<<"${topics}"; then
    break
  fi
  sleep 1
done

if ! grep -Fqx /ground_truth/odom <<<"${topics:-}"; then
  echo "Gazebo/bridge topics did not become ready" >&2
  exit 4
fi

docker exec -d "${container}" bash -lc \
  "source /opt/ros/humble/setup.bash;
   source /workspaces/isaac_ros-dev/install/setup.bash;
   export ROS_DOMAIN_ID=43;
   exec ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_d435i_sim.launch.py enable_imu_fusion:=false enable_debug_mode:=true debug_dump_path:=${container_output}/debug_dump >${container_output}/cuvslam.log 2>&1"

ready=0
for _ in $(seq 1 30); do
  if docker exec "${container}" bash -lc \
      "source /opt/ros/humble/setup.bash;
       source /workspaces/isaac_ros-dev/install/setup.bash;
       export ROS_DOMAIN_ID=43;
       export ROS2CLI_DISABLE_DAEMON=1;
       timeout 5 ros2 topic info /visual_slam/status" \
      >"${host_output}/ready_topic_info.txt" 2>/dev/null &&
      grep -Eq 'Publisher count:[[:space:]]*1' \
        "${host_output}/ready_topic_info.txt"; then
    ready=1
    break
  fi
  sleep 1
done

if [[ "${ready}" -ne 1 ]]; then
  echo "cuVSLAM status publisher did not become ready" >&2
  exit 5
fi

docker exec "${container}" bash \
  /workspaces/isaac_ros-dev/tools/run_container_1m_trial.sh \
  "${container_output}"

printf '%s\n' "${host_output}" > /home/tseng/isaac_ros_ws/data/latest_1m_acceptance_path.txt
printf '%s\n' "${world}" > "${host_output}/world_path.txt"
echo "HOST_OUTPUT=${host_output}"
echo "JETSON_1M_ACCEPTANCE_COMPLETE"
