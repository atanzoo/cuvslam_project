#!/usr/bin/env bash
set -Eeo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 OUTPUT_DIR" >&2
  exit 2
fi

output_dir=$1
motion_profile="${CUVSLAM_MOTION_PROFILE:-turn_only}"
turn_rate_deg_s="${CUVSLAM_TURN_RATE_DEG_S:-20.0}"
linear_speed="${CUVSLAM_LINEAR_SPEED:-0.12}"
pre_turn_settle="${CUVSLAM_PRE_TURN_SETTLE:-8}"
phase_settle="${CUVSLAM_PHASE_SETTLE:-2}"
post_motion_settle="${CUVSLAM_POST_MOTION_SETTLE:-3}"
stop_distance="${CUVSLAM_STOP_DISTANCE:-0.30}"
loop_leg_1="${CUVSLAM_LOOP_LEG_1:-1.0}"
loop_leg_2="${CUVSLAM_LOOP_LEG_2:-0.5}"
loop_leg_3="${CUVSLAM_LOOP_LEG_3:-1.0}"
loop_leg_4="${CUVSLAM_LOOP_LEG_4:-0.5}"
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

timeout 30 ros2 topic echo --once /visual_slam/status \
  >"${output_dir}/initial_status.txt"

if [[ -n "${CUVSLAM_POSE_ONLY:-}" ]]; then
  bag_topics=(
    /visual_slam/tracking/odometry
    /visual_slam/tracking/vo_pose
    /visual_slam/tracking/slam_path
    /visual_slam/status
    # NVIDIA documents these PoseGraph outputs as the closest ROS-level
    # observable of the keyframe graph.  Keep them in pose-only trials so
    # keyframe timing can be compared with native motion without recording
    # the large stereo image/point-cloud streams.
    /visual_slam/vis/pose_graph_nodes
    /visual_slam/vis/pose_graph_edges
    /simulation/native_pose
    /tf
    /tf_static
    /d435i/infra1/camera_info
    /d435i/infra2/camera_info
    /cuvslam/input/infra2/camera_info
    /d435i/imu
    /clock
  )
else
  bag_topics=(
    /visual_slam/tracking/odometry
    /visual_slam/tracking/vo_pose
    /visual_slam/tracking/slam_path
    /visual_slam/status
    /visual_slam/vis/observations_cloud
    /visual_slam/vis/landmarks_cloud
    /simulation/native_pose
    /simulation/ground_truth_world_markers
    /tf
    /tf_static
    /d435i/infra1/image_rect_raw
    /d435i/infra2/image_rect_raw
    /d435i/infra1/camera_info
    /d435i/infra2/camera_info
    /d435i/imu
    /cuvslam/input/infra2/camera_info
    /clock
  )
fi

ros2 bag record --output "${output_dir}/bag" "${bag_topics[@]}" \
  >"${output_dir}/bag.log" 2>&1 &
bag_pid=$!

sleep 3
if ! kill -0 "${bag_pid}" 2>/dev/null; then
  echo "rosbag recorder exited before motion" >&2
  exit 1
fi

if [[ "${motion_profile}" == "turn_only" ]]; then
  python3 /workspaces/isaac_ros-dev/simulation/cuvslam/tools/simulation_imu_turn_profile.py \
    --target-angle-deg 90 \
    --turn-rate-deg-s "${turn_rate_deg_s}" \
    --stop-distance 0.35 \
    --pre-turn-settle "${pre_turn_settle}" \
    --timeout 120 \
    | tee "${output_dir}/motion.log"
elif [[ "${motion_profile}" == "native_corner" ]]; then
  python3 /workspaces/isaac_ros-dev/simulation/cuvslam/tools/simulation_native_corner_profile.py \
    --first-leg-distance 1.0 \
    --second-leg-distance 0.5 \
    --speed "${linear_speed}" \
    --target-angle-deg 90 \
    --turn-rate-deg-s "${turn_rate_deg_s}" \
    --stop-distance "${stop_distance}" \
    --pre-motion-settle "${pre_turn_settle}" \
    --phase-settle "${phase_settle}" \
    --timeout 120 \
    | tee "${output_dir}/motion.log"
elif [[ "${motion_profile}" == "native_loop" ]]; then
  python3 /workspaces/isaac_ros-dev/simulation/cuvslam/tools/simulation_native_loop_profile.py \
    --leg-distances "${loop_leg_1}" "${loop_leg_2}" "${loop_leg_3}" "${loop_leg_4}" \
    --speed "${linear_speed}" \
    --target-angle-deg 90 \
    --turn-rate-deg-s "${turn_rate_deg_s}" \
    --stop-distance "${stop_distance}" \
    --pre-motion-settle "${pre_turn_settle}" \
    --phase-settle "${phase_settle}" \
    --timeout 120 \
    | tee "${output_dir}/motion.log"
else
  echo "unsupported motion profile (wheel odometry is prohibited): ${motion_profile}" >&2
  exit 2
fi

publish_zero
sleep "${post_motion_settle}"
stop_bag
echo "CONTAINER_STRAIGHT_MAPPING_COMPLETE"
