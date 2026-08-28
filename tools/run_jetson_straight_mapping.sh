#!/usr/bin/env bash
set -Eeuo pipefail

container=isaac_ros_dev-aarch64-container
gazebo_unit=cuvslam-straight-map-gazebo.service
bridge_unit=cuvslam-straight-map-bridge.service
test_label="${CUVSLAM_TEST_LABEL:-straight_mapping_observable}"
enable_observations_view="${CUVSLAM_ENABLE_OBSERVATIONS_VIEW:-false}"
enable_slam_visualization="${CUVSLAM_ENABLE_SLAM_VISUALIZATION:-true}"
enable_imu_fusion="${CUVSLAM_ENABLE_IMU_FUSION:-false}"
enable_mapping="${CUVSLAM_ENABLE_MAPPING:-true}"
motion_profile="${CUVSLAM_MOTION_PROFILE:-turn_only}"
turn_rate_deg_s="${CUVSLAM_TURN_RATE_DEG_S:-20.0}"
linear_speed="${CUVSLAM_LINEAR_SPEED:-0.12}"
stop_distance="${CUVSLAM_STOP_DISTANCE:-0.30}"
pre_turn_settle="${CUVSLAM_PRE_TURN_SETTLE:-8}"
phase_settle="${CUVSLAM_PHASE_SETTLE:-2}"
camera_x="${CUVSLAM_CAMERA_X:-0.19}"
post_motion_settle="${CUVSLAM_POST_MOTION_SETTLE:-3}"
pose_only="${CUVSLAM_POSE_ONLY:-}"
loop_leg_1="${CUVSLAM_LOOP_LEG_1:-1.0}"
loop_leg_2="${CUVSLAM_LOOP_LEG_2:-0.5}"
loop_leg_3="${CUVSLAM_LOOP_LEG_3:-1.0}"
loop_leg_4="${CUVSLAM_LOOP_LEG_4:-0.5}"
run_id="${test_label}_20260731_$$"
partition="${CUVSLAM_TEST_PARTITION:-${run_id}}"
world="${CUVSLAM_TEST_WORLD:-/home/tseng/jetson_slam_ws/install/slam_gazebo/share/slam_gazebo/worlds/cuvslam_mapping_observable.sdf}"
host_output="/home/tseng/isaac_ros_ws/data/${run_id}"
container_output="/workspaces/isaac_ros-dev/data/${run_id}"
container_world="${container_output}/active_world.sdf"
native_pose_pid=
native_pose_relay_pid=

stop_native_pose() {
  if [[ -n "${native_pose_pid}" ]] &&
      kill -0 "${native_pose_pid}" 2>/dev/null; then
    kill -INT "${native_pose_pid}" || true
    wait "${native_pose_pid}" || true
  fi
  native_pose_pid=
}

stop_native_pose_relay() {
  if [[ -n "${native_pose_relay_pid}" ]] &&
      kill -0 "${native_pose_relay_pid}" 2>/dev/null; then
    kill -TERM "${native_pose_relay_pid}" || true
    for _ in $(seq 1 20); do
      kill -0 "${native_pose_relay_pid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${native_pose_relay_pid}" 2>/dev/null; then
      kill -KILL "${native_pose_relay_pid}" || true
    fi
    wait "${native_pose_relay_pid}" 2>/dev/null || true
  fi
  native_pose_relay_pid=
}

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
  stop_native_pose_relay
  stop_native_pose
  systemctl --user stop "${bridge_unit}" "${gazebo_unit}"
  sleep 4
  docker stop "${container}" >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

if ps -eo cmd | grep -E \
    '[i]gn gazebo|[p]arameter_bridge|[r]os2 bag record|[s]imulation_truth_.*profile' \
    >/dev/null; then
  echo "another simulation or recorder process is active" >&2
  exit 3
fi

docker start "${container}" >/dev/null
mkdir -p "${host_output}"
cp -f "${world}" "${host_output}/active_world.sdf"

systemd-run --user \
  --unit="${gazebo_unit%.service}" \
  --collect \
  --property=KillMode=control-group \
  --property=TimeoutStopSec=8s \
  /bin/bash -lc \
  "source /opt/ros/foxy/setup.bash;
   source /home/tseng/jetson_slam_ws/install/setup.bash;
   export ROS_DOMAIN_ID=43;
   exec ros2 launch slam_gazebo gz_sim.launch.py world:=${world} ign_partition:=${partition} use_gui:=false >${host_output}/gazebo.log 2>&1"

systemd-run --user \
  --unit="${bridge_unit%.service}" \
  --collect \
  --property=KillMode=control-group \
  --property=TimeoutStopSec=8s \
  /bin/bash -lc \
  "source /opt/ros/foxy/setup.bash;
   source /home/tseng/jetson_slam_ws/install/setup.bash;
   export ROS_DOMAIN_ID=43;
   exec ros2 launch slam_gazebo gz_bridge_portable.launch.py ign_partition:=${partition} enable_native_pose:=true >${host_output}/bridge.log 2>&1"

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
      grep -Fqx /d435i/imu <<<"${topics}"; then
    break
  fi
  sleep 1
done

if ! grep -Fqx /d435i/imu <<<"${topics:-}"; then
  echo "Gazebo/bridge topics did not become ready" >&2
  exit 4
fi
if grep -Fqx /ground_truth/odom <<<"${topics:-}"; then
  echo "prohibited wheel odometry topic is present: /ground_truth/odom" >&2
  exit 8
fi

bash -lc \
  "source /opt/ros/foxy/setup.bash;
   export ROS_DOMAIN_ID=43;
   exec python3 /home/tseng/isaac_ros_ws/tools/simulation_native_pose_relay.py" \
  >"${host_output}/native_pose_relay.log" 2>&1 &
native_pose_relay_pid=$!

native_pose_topic="$(
  env IGN_PARTITION="${partition}" ign topic -l |
    awk '/\/dynamic_pose\/info$/ {print; exit}'
)"
if [[ -z "${native_pose_topic}" ]]; then
  echo "Gazebo native dynamic pose topic did not become ready" >&2
  exit 7
fi
env IGN_PARTITION="${partition}" ign topic \
  -e -t "${native_pose_topic}" \
  >"${host_output}/native_dynamic_pose.log" 2>&1 &
native_pose_pid=$!

if ! bash -lc \
  "source /opt/ros/foxy/setup.bash;
   export ROS_DOMAIN_ID=43;
   export ROS2CLI_DISABLE_DAEMON=1;
   timeout 8 ros2 topic echo /d435i/infra1/image_rect_raw --no-arr \
     >${host_output}/first_left_image.txt 2>&1 || true;
   grep -q '^header:' ${host_output}/first_left_image.txt"; then
  echo "Gazebo camera topic exists but produced no image" >&2
  exit 6
fi

docker exec -d "${container}" bash -lc \
  "source /opt/ros/humble/setup.bash;
   source /workspaces/isaac_ros-dev/install/setup.bash;
   export ROS_DOMAIN_ID=43;
   exec ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_d435i_sim.launch.py camera_x:=${camera_x} enable_mapping:=${enable_mapping} enable_imu_fusion:=${enable_imu_fusion} enable_debug_mode:=false enable_slam_visualization:=${enable_slam_visualization} enable_observations_view:=${enable_observations_view} world_sdf:=${container_world} >${container_output}/cuvslam.log 2>&1"

ready_topic="/visual_slam/vis/landmarks_cloud"
if [[ "${enable_slam_visualization}" != "true" ]]; then
  ready_topic="/visual_slam/tracking/vo_pose"
fi
ready=0
for _ in $(seq 1 30); do
  if docker exec "${container}" bash -lc \
      "source /opt/ros/humble/setup.bash;
       source /workspaces/isaac_ros-dev/install/setup.bash;
       export ROS_DOMAIN_ID=43;
       export ROS2CLI_DISABLE_DAEMON=1;
       timeout 5 ros2 topic info ${ready_topic}" \
      >"${host_output}/ready_landmark_topic_info.txt" 2>/dev/null &&
      grep -Eq 'Publisher count:[[:space:]]*1' \
        "${host_output}/ready_landmark_topic_info.txt"; then
    ready=1
    break
  fi
  sleep 1
done

if [[ "${ready}" -ne 1 ]]; then
  echo "landmark publisher did not become ready" >&2
  exit 5
fi

docker exec \
  --env CUVSLAM_MOTION_PROFILE="${motion_profile}" \
  --env CUVSLAM_TURN_RATE_DEG_S="${turn_rate_deg_s}" \
  --env CUVSLAM_LINEAR_SPEED="${linear_speed}" \
  --env CUVSLAM_STOP_DISTANCE="${stop_distance}" \
  --env CUVSLAM_PRE_TURN_SETTLE="${pre_turn_settle}" \
  --env CUVSLAM_PHASE_SETTLE="${phase_settle}" \
  --env CUVSLAM_POST_MOTION_SETTLE="${post_motion_settle}" \
  --env CUVSLAM_POSE_ONLY="${pose_only}" \
  --env CUVSLAM_LOOP_LEG_1="${loop_leg_1}" \
  --env CUVSLAM_LOOP_LEG_2="${loop_leg_2}" \
  --env CUVSLAM_LOOP_LEG_3="${loop_leg_3}" \
  --env CUVSLAM_LOOP_LEG_4="${loop_leg_4}" \
  "${container}" bash \
  /workspaces/isaac_ros-dev/tools/run_container_straight_mapping_trial.sh \
  "${container_output}"

stop_native_pose
stop_native_pose_relay
printf '%s\n' "${host_output}" > /home/tseng/isaac_ros_ws/data/latest_straight_mapping_path.txt
printf '%s\n' "${world}" > "${host_output}/world_path.txt"
printf '%s\n' "${enable_imu_fusion}" > "${host_output}/enable_imu_fusion.txt"
printf '%s\n' "${enable_mapping}" > "${host_output}/enable_mapping.txt"
printf '%s\n' "${camera_x}" > "${host_output}/camera_x.txt"
echo "HOST_OUTPUT=${host_output}"
echo "JETSON_STRAIGHT_MAPPING_COMPLETE"
