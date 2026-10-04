#!/usr/bin/env bash
set -eo pipefail

# Run this inside the Jetson Isaac ROS container before selecting ekf_shadow.
# It is intentionally a check only: it never installs or changes packages.

source /opt/ros/humble/setup.bash
if [[ -f /workspaces/isaac_ros-dev/install/setup.bash ]]; then
  source /workspaces/isaac_ros-dev/install/setup.bash
fi

# The base image's ROS setup references optional environment variables that
# are not defined in a non-interactive shell.  Enable nounset only after both
# setup files have been sourced so preflight reports dependency state rather
# than failing during shell initialization.
set -u

missing=0
for package in rf2o_laser_odometry robot_localization; do
  if prefix="$(ros2 pkg prefix "${package}" 2>/dev/null)"; then
    echo "dependency_ok package=${package} prefix=${prefix}"
  else
    echo "dependency_missing package=${package}" >&2
    missing=1
  fi
done

if (( missing )); then
  echo "preflight_failed reason=missing_lidar_odom_or_ekf_dependency" >&2
  exit 1
fi

echo "executable_check rf2o_laser_odometry"
ros2 pkg executables rf2o_laser_odometry
echo "executable_check robot_localization"
ros2 pkg executables robot_localization
echo "preflight_ok mode=ekf_shadow tf_owner=cuvslam"
