#!/usr/bin/env python3
"""Analyze real cuVSLAM R2 bags for in-place-turn false translation."""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter


ODOM_TOPIC = "/visual_slam/tracking/odometry"
STATUS_TOPIC = "/visual_slam/status"
IMU_TOPIC = "/camera/imu"


def stamp_ns(message, fallback: int) -> int:
    header = getattr(message, "header", None)
    if header is None:
        return fallback
    value = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
    return value or fallback


def yaw_degrees(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def wrapped_delta_degrees(current: float, previous: float) -> float:
    return (current - previous + 180.0) % 360.0 - 180.0


def analyze_turns(poses: list[tuple[int, float, float, float]]) -> list[dict[str, float | int | str]]:
    """Find continuous yaw-rate segments and measure their XY estimate."""
    if len(poses) < 3:
        return []
    threshold_deg_s = 2.0
    timeout_ns = 250_000_000
    active = False
    quiet_since_ns: int | None = None
    start: tuple[int, float, float, float] | None = None
    last_active: tuple[int, float, float, float] | None = None
    max_xy = 0.0
    path_length = 0.0
    turns: list[dict[str, float | int | str]] = []

    def finish() -> None:
        nonlocal active, quiet_since_ns, start, last_active, max_xy, path_length
        if not active or start is None or last_active is None:
            return
        yaw_delta = abs(wrapped_delta_degrees(last_active[3], start[3]))
        if yaw_delta >= 30.0:
            candidate_status = "PASS" if max_xy <= 0.10 else ("REVIEW" if max_xy <= 0.20 else "FAIL")
            turns.append({
                "status": "待確認",
                "candidate_status": candidate_status,
                "yaw_delta_deg": yaw_delta,
                "max_xy_displacement_m": max_xy,
                "path_length_m": path_length,
                "start_t_ns": start[0],
                "end_t_ns": last_active[0],
                "start_t_s": start[0] / 1e9,
                "end_t_s": last_active[0] / 1e9,
            })
        active = False
        quiet_since_ns = None
        start = None
        last_active = None
        max_xy = 0.0
        path_length = 0.0

    previous = poses[0]
    for current in poses[1:]:
        dt_s = (current[0] - previous[0]) / 1e9
        if dt_s <= 0.0:
            continue
        yaw_rate = wrapped_delta_degrees(current[3], previous[3]) / dt_s
        if abs(yaw_rate) >= threshold_deg_s:
            if not active:
                active = True
                quiet_since_ns = None
                start = current
                last_active = current
                max_xy = 0.0
                path_length = 0.0
            else:
                assert last_active is not None
                path_length += math.hypot(current[1] - last_active[1], current[2] - last_active[2])
                last_active = current
            assert start is not None
            max_xy = max(max_xy, math.hypot(current[1] - start[1], current[2] - start[2]))
            quiet_since_ns = None
        elif active:
            if quiet_since_ns is None:
                quiet_since_ns = current[0]
            elif current[0] - quiet_since_ns >= timeout_ns:
                finish()
        previous = current
    finish()
    return turns


def integrate_imu(
    samples: list[tuple[int, float, float, float]],
    start_ns: int,
    end_ns: int,
) -> dict[str, list[float] | int]:
    selected = [sample for sample in samples if start_ns <= sample[0] <= end_ns]
    signed = [0.0, 0.0, 0.0]
    absolute = [0.0, 0.0, 0.0]
    for previous, current in zip(selected, selected[1:]):
        dt_s = (current[0] - previous[0]) / 1e9
        if dt_s <= 0.0 or dt_s > 0.1:
            continue
        for axis in range(3):
            average = 0.5 * (previous[axis + 1] + current[axis + 1])
            signed[axis] += average * dt_s
            absolute[axis] += abs(average) * dt_s
    return {
        "samples": len(selected),
        "signed_deg": [math.degrees(value) for value in signed],
        "absolute_deg": [math.degrees(value) for value in absolute],
    }


def main() -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = {ODOM_TOPIC, STATUS_TOPIC}
    missing = sorted(required - topic_types.keys())
    if missing:
        raise RuntimeError(f"bag missing topics: {', '.join(missing)}")
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in (ODOM_TOPIC, STATUS_TOPIC, IMU_TOPIC)
        if topic in topic_types
    }

    poses: list[tuple[int, float, float, float]] = []
    statuses: Counter[str] = Counter()
    imu_samples: list[tuple[int, float, float, float]] = []
    imu_gyro_peak = 0.0
    while reader.has_next():
        topic, data, bag_stamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == ODOM_TOPIC:
            pose = message.pose.pose
            poses.append((
                stamp_ns(message, bag_stamp_ns),
                float(pose.position.x),
                float(pose.position.y),
                yaw_degrees(pose.orientation),
            ))
        elif topic == STATUS_TOPIC:
            statuses[str(message.vo_state)] += 1
        else:
            current_stamp_ns = stamp_ns(message, bag_stamp_ns)
            gyro = message.angular_velocity
            imu_samples.append((current_stamp_ns, float(gyro.x), float(gyro.y), float(gyro.z)))
            imu_gyro_peak = max(imu_gyro_peak, math.sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z))

    poses.sort(key=lambda item: item[0])
    imu_samples.sort(key=lambda item: item[0])
    if len(poses) < 2:
        raise RuntimeError("not enough odometry samples")
    first, last = poses[0], poses[-1]
    path_length = sum(
        math.hypot(current[1] - previous[1], current[2] - previous[2])
        for previous, current in zip(poses, poses[1:])
    )
    turns = analyze_turns(poses)
    for turn in turns:
        turn["imu_rotation"] = integrate_imu(
            imu_samples,
            int(turn.pop("start_t_ns")),
            int(turn.pop("end_t_ns")),
        )
    print(json.dumps({
        "odom_samples": len(poses),
        "duration_s": (last[0] - first[0]) / 1e9,
        "start_position_xy": [first[1], first[2]],
        "end_position_xy": [last[1], last[2]],
        "end_to_start_xy_m": math.hypot(last[1] - first[1], last[2] - first[2]),
        "odom_yaw_delta_deg": wrapped_delta_degrees(last[3], first[3]),
        "odom_path_length_m": path_length,
        "vo_state_counts": dict(statuses),
        "imu_samples": len(imu_samples),
        "imu_gyro_peak_rad_s": imu_gyro_peak,
        "turns": turns,
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
