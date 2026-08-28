#!/usr/bin/env bash
set -Eeo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 OUTPUT_DIR" >&2
  exit 2
fi

output_dir=$1
bag_pid=

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
set -u
export ROS_DOMAIN_ID=43
export ROS2CLI_DISABLE_DAEMON=1

mkdir -p "${output_dir}"

publish_zero() {
  timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
}

stop_bag() {
  if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
    kill -INT "${bag_pid}"
    wait "${bag_pid}" || true
  fi
  bag_pid=
}

cleanup() {
  publish_zero
  stop_bag
}
trap cleanup EXIT INT TERM

timeout 15 ros2 topic echo --once /visual_slam/status \
  >"${output_dir}/initial_status.txt"

ros2 bag record \
  --output "${output_dir}/bag" \
  /ground_truth/odom \
  /visual_slam/tracking/odometry \
  /visual_slam/status \
  /tf \
  /tf_static \
  /d435i/infra1/image_rect_raw \
  /d435i/infra2/image_rect_raw \
  /d435i/infra1/camera_info \
  /d435i/infra2/camera_info \
  /clock \
  >"${output_dir}/bag.log" 2>&1 &
bag_pid=$!

sleep 3
if ! kill -0 "${bag_pid}" 2>/dev/null; then
  echo "rosbag recorder exited before motion" >&2
  exit 1
fi

python3 /workspaces/isaac_ros-dev/tools/simulation_truth_distance_profile.py \
  --target-distance 1.0 \
  --speed 0.12 \
  --stop-distance 0.35 \
  --timeout 35 \
  | tee "${output_dir}/motion.log"

publish_zero
stop_bag
echo "CONTAINER_TRIAL_COMPLETE"
