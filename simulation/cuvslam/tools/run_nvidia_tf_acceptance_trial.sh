#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: $0 OUTPUT_DIR TRIAL_NAME PROFILE_SCRIPT" >&2
  exit 2
fi

output_dir=$1
trial_name=$2
profile_script=$3
recorder_pid=

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
set -u
export ROS_DOMAIN_ID=43
export ROS2CLI_DISABLE_DAEMON=1

trial_dir="${output_dir}/${trial_name}"
mkdir -p "${trial_dir}"

publish_zero() {
  timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
}

stop_recorder() {
  if [[ -n "${recorder_pid}" ]] && kill -0 "${recorder_pid}" 2>/dev/null; then
    kill -INT "${recorder_pid}"
    wait "${recorder_pid}" || true
  fi
  recorder_pid=
}

cleanup() {
  publish_zero
  stop_recorder
}
trap cleanup EXIT INT TERM

start_recorder() {
  local phase=$1
  ros2 bag record \
    --output "${trial_dir}/${phase}/bag" \
    /ground_truth/odom \
    /visual_slam/tracking/odometry \
    /visual_slam/status \
    /tf \
    /tf_static \
    >"${trial_dir}/${phase}_recorder.log" 2>&1 &
  recorder_pid=$!
  sleep 2
  if ! kill -0 "${recorder_pid}" 2>/dev/null; then
    echo "recorder failed during ${phase}" >&2
    exit 1
  fi
}

echo "trial=${trial_name}: static phase"
mkdir -p "${trial_dir}/static"
start_recorder static
publish_zero
sleep 10
stop_recorder

echo "trial=${trial_name}: straight phase"
mkdir -p "${trial_dir}/straight"
start_recorder straight
python3 "${profile_script}" \
  --mode straight \
  --speed 0.12 \
  --leg-duration 5.0 \
  --stop-distance 0.35 \
  | tee "${trial_dir}/straight_profile.log"
stop_recorder

echo "trial=${trial_name}: turn phase"
mkdir -p "${trial_dir}/turn"
start_recorder turn
python3 "${profile_script}" \
  --mode turn \
  --turn-rate -0.12 \
  --turn-duration 5.0 \
  --stop-distance 0.35 \
  | tee "${trial_dir}/turn_profile.log"
stop_recorder

publish_zero
echo "trial=${trial_name}: complete"
