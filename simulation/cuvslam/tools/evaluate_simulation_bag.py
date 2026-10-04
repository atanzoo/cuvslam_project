#!/usr/bin/env python3
"""Report closure and tracking evidence from a ROS 2 simulation bag."""

import argparse
import math
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


POSE_TOPICS = (
    "/ground_truth/odom",
    "/visual_slam/tracking/odometry",
    "/visual_slam/tracking/vo_pose",
    "/visual_slam/tracking/slam_path",
)
STATUS_TOPIC = "/visual_slam/status"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in (*POSE_TOPICS, STATUS_TOPIC)
    }
    endpoints = {topic: [] for topic in POSE_TOPICS}
    status_counts = Counter()

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == STATUS_TOPIC:
            status_counts[message.vo_state] += 1
            continue
        if topic == "/visual_slam/tracking/vo_pose":
            position = message.pose.position
            orientation = message.pose.orientation
        elif topic == "/visual_slam/tracking/slam_path":
            if not message.poses:
                continue
            position = message.poses[-1].pose.position
            orientation = message.poses[-1].pose.orientation
        else:
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
        point = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if len(endpoints[topic]) == 0:
            endpoints[topic].append(point)
        elif len(endpoints[topic]) == 1:
            endpoints[topic].append(point)
        else:
            endpoints[topic][1] = point

    for topic in POSE_TOPICS:
        start, end = endpoints[topic]
        closure_2d = math.hypot(end[0] - start[0], end[1] - start[1])
        displacement_3d = math.sqrt(
            (end[0] - start[0]) ** 2
            + (end[1] - start[1]) ** 2
            + (end[2] - start[2]) ** 2
        )
        quaternion_dot = abs(sum(a * b for a, b in zip(start[3:], end[3:])))
        quaternion_dot = max(-1.0, min(1.0, quaternion_dot))
        rotation_degrees = math.degrees(2.0 * math.acos(quaternion_dot))
        print(
            f"{topic}: start=({start[0]:.4f}, {start[1]:.4f}), "
            f"end=({end[0]:.4f}, {end[1]:.4f}), "
            f"delta_z={end[2] - start[2]:.4f}m, "
            f"displacement_2d={closure_2d:.4f}m, "
            f"displacement_3d={displacement_3d:.4f}m, "
            f"rotation={rotation_degrees:.2f}deg"
        )
    print(
        "vo_state_counts: "
        + ", ".join(
            f"{state}={count}" for state, count in sorted(status_counts.items())
        )
    )


if __name__ == "__main__":
    main()
