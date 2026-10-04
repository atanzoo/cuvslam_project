#!/usr/bin/env python3
"""Summarize real cuVSLAM mapping/backend evidence from a ROS 2 bag.

This intentionally does not claim metric map accuracy because a real bag has
no ground-truth pose.  It reports closure, graph growth, map->odom correction,
and whether the backend visualization topics contain actual data.
"""

from __future__ import annotations

import argparse
import bisect
import math
import statistics


ODOM_TOPIC = "/visual_slam/tracking/odometry"
VO_TOPIC = "/visual_slam/tracking/vo_pose"
SLAM_PATH_TOPIC = "/visual_slam/tracking/slam_path"
TF_TOPIC = "/tf"
NODES_TOPIC = "/visual_slam/vis/pose_graph_nodes"
EDGES_TOPIC = "/visual_slam/vis/pose_graph_edges"
LOOP_TOPIC = "/visual_slam/vis/loop_closure_cloud"
LANDMARK_TOPIC = "/visual_slam/vis/landmarks_cloud"
OBSERVATION_TOPIC = "/visual_slam/vis/observations_cloud"


def yaw_degrees(orientation) -> float:
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    return math.degrees(
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    )


def wrapped_degrees(value: float) -> float:
    return math.degrees(math.atan2(math.sin(math.radians(value)), math.cos(math.radians(value))))


def pose_values(pose) -> tuple[float, float, float, float]:
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        yaw_degrees(pose.orientation),
    )


def transform_values(transform) -> tuple[float, float, float, float]:
    return (
        float(transform.translation.x),
        float(transform.translation.y),
        float(transform.translation.z),
        yaw_degrees(transform.rotation),
    )


def pose_delta(first, last) -> tuple[float, float, float, float]:
    dx = last[0] - first[0]
    dy = last[1] - first[1]
    dz = last[2] - first[2]
    return math.hypot(dx, dy), math.sqrt(dx * dx + dy * dy + dz * dz), dz, wrapped_degrees(last[3] - first[3])


def cloud_points(message) -> int:
    return int(message.width) * int(message.height)


def nearest(samples, stamps: list[int], target_ns: int):
    index = bisect.bisect_left(stamps, target_ns)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    return min(candidates, key=lambda item: abs(item[0] - target_ns))


def count_summary(name: str, values: list[int]) -> None:
    if not values:
        print(f"{name}: messages=0")
        return
    nonempty = [value for value in values if value > 0]
    print(
        f"{name}: messages={len(values)} nonempty={len(nonempty)} "
        f"first={values[0]} last={values[-1]} max={max(values)} "
        f"median={statistics.median(values):.1f}"
    )


def main() -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    selected_topics = {
        ODOM_TOPIC,
        VO_TOPIC,
        SLAM_PATH_TOPIC,
        TF_TOPIC,
        NODES_TOPIC,
        EDGES_TOPIC,
        LOOP_TOPIC,
        LANDMARK_TOPIC,
        OBSERVATION_TOPIC,
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    available = selected_topics & topic_types.keys()
    message_types = {topic: get_message(topic_types[topic]) for topic in available}

    odom = []
    vo = []
    slam = []
    slam_path_lengths: list[int] = []
    path_revision_updates = 0
    path_revision_max_m = 0.0
    previous_path = []
    map_odom = []
    nodes: list[int] = []
    edges: list[int] = []
    loops: list[int] = []
    landmarks: list[int] = []
    observations: list[int] = []
    frames: dict[str, set[str]] = {}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == ODOM_TOPIC:
            frames.setdefault(topic, set()).add(message.header.frame_id)
            odom.append((timestamp_ns, pose_values(message.pose.pose)))
        elif topic == VO_TOPIC:
            frames.setdefault(topic, set()).add(message.header.frame_id)
            vo.append((timestamp_ns, pose_values(message.pose)))
        elif topic == SLAM_PATH_TOPIC:
            frames.setdefault(topic, set()).add(message.header.frame_id)
            current_path = [pose_values(item.pose) for item in message.poses]
            slam_path_lengths.append(len(current_path))
            if current_path:
                slam.append((timestamp_ns, current_path[-1]))
            common = min(len(previous_path), len(current_path))
            revised = False
            # A Path update can contain the complete pose history at 30 Hz.
            # Uniformly sample the shared history so long real bags remain
            # inexpensive to audit on Jetson while still detecting global
            # backend rewrites.
            stride = max(1, common // 64)
            indices = list(range(0, common, stride))
            if common and (not indices or indices[-1] != common - 1):
                indices.append(common - 1)
            for index in indices:
                delta_xy, delta_3d, _, delta_yaw = pose_delta(previous_path[index], current_path[index])
                path_revision_max_m = max(path_revision_max_m, delta_3d)
                if delta_xy > 0.001 or abs(delta_yaw) > 0.05:
                    revised = True
            if revised:
                path_revision_updates += 1
            previous_path = current_path
        elif topic == TF_TOPIC:
            for item in message.transforms:
                parent = item.header.frame_id.lstrip("/")
                child = item.child_frame_id.lstrip("/")
                if parent == "map" and child == "odom":
                    map_odom.append((timestamp_ns, transform_values(item.transform)))
        elif topic == NODES_TOPIC:
            nodes.append(len(message.poses))
        elif topic == EDGES_TOPIC:
            # Marker LINE_LIST stores two points per graph edge.
            edges.append(len(message.points) // 2 if int(message.type) == 5 else len(message.points))
        elif topic == LOOP_TOPIC:
            loops.append(cloud_points(message))
        elif topic == LANDMARK_TOPIC:
            landmarks.append(cloud_points(message))
        elif topic == OBSERVATION_TOPIC:
            observations.append(cloud_points(message))

    for samples in (odom, vo, slam, map_odom):
        samples.sort(key=lambda item: item[0])

    print(f"bag={args.bag}")
    for topic in sorted(frames):
        print(f"frame {topic}: {','.join(sorted(frames[topic])) or '<empty>'}")
    for name, samples in (("tracking_odom", odom), ("vo_pose", vo), ("slam_path_tip", slam)):
        if len(samples) < 2:
            print(f"{name}: samples={len(samples)}")
            continue
        xy, distance_3d, dz, yaw = pose_delta(samples[0][1], samples[-1][1])
        print(
            f"{name}: samples={len(samples)} closure_xy={xy:.4f}m "
            f"closure_3d={distance_3d:.4f}m dz={dz:+.4f}m yaw={yaw:+.2f}deg"
        )

    if slam_path_lengths:
        print(
            f"slam_path: updates={len(slam_path_lengths)} poses="
            f"{slam_path_lengths[0]}->{slam_path_lengths[-1]} max={max(slam_path_lengths)} "
            f"historical_revision_updates={path_revision_updates} "
            f"max_historical_pose_revision={path_revision_max_m:.4f}m"
        )
    if map_odom:
        xy, distance_3d, dz, yaw = pose_delta(map_odom[0][1], map_odom[-1][1])
        print(
            f"map_to_odom: samples={len(map_odom)} change_xy={xy:.4f}m "
            f"change_3d={distance_3d:.4f}m dz={dz:+.4f}m yaw={yaw:+.2f}deg"
        )
    else:
        print("map_to_odom: samples=0")

    if odom and slam:
        odom_stamps = [item[0] for item in odom]
        differences = []
        yaw_differences = []
        for stamp, slam_pose in slam:
            _, odom_pose = nearest(odom, odom_stamps, stamp)
            xy, _, _, yaw = pose_delta(odom_pose, slam_pose)
            differences.append(xy)
            yaw_differences.append(abs(yaw))
        print(
            f"slam_tip_vs_nearest_odom: comparisons={len(differences)} "
            f"final_xy={differences[-1]:.4f}m max_xy={max(differences):.4f}m "
            f"final_yaw={yaw_differences[-1]:.2f}deg max_yaw={max(yaw_differences):.2f}deg"
        )

    count_summary("pose_graph_nodes", nodes)
    count_summary("pose_graph_edges", edges)
    count_summary("loop_closure_cloud", loops)
    count_summary("landmarks_cloud", landmarks)
    count_summary("observations_cloud", observations)

    graph_growth = bool(nodes and max(nodes) > min(nodes))
    graph_edges = bool(edges and max(edges) > 0)
    nonempty_loop = any(value > 0 for value in loops)
    backend_correction = bool(
        path_revision_updates
        or (map_odom and pose_delta(map_odom[0][1], map_odom[-1][1])[1] > 0.001)
    )
    print(
        "backend_evidence: "
        f"graph_growth={str(graph_growth).lower()} "
        f"graph_edges={str(graph_edges).lower()} "
        f"nonempty_loop_cloud={str(nonempty_loop).lower()} "
        f"path_or_map_correction={str(backend_correction).lower()}"
    )
    print("note: backend evidence is not ground-truth map accuracy")


if __name__ == "__main__":
    main()
