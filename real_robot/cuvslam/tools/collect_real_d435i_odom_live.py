#!/usr/bin/env python3
"""Stream lightweight real-D435i odometry and 2D LiDAR telemetry for the GUI."""

from __future__ import annotations

import json
import math
import base64
import hashlib
import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan
import tf2_ros


def yaw_degrees(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def yaw_radians(q) -> float:
    """Return the planar yaw component of a ROS quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def stamp_nanoseconds(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def diagnostic_level_value(value) -> int | None:
    """Normalize ROS uint8 fields across Python message bindings.

    Some ROS 2 Python bindings expose DiagnosticStatus.level as a one-byte
    bytes value instead of an integer.  ``int(b"\x01")`` raises ValueError,
    so consume the byte value explicitly and keep malformed diagnostics from
    terminating the live telemetry collector.
    """
    if isinstance(value, (bytes, bytearray, memoryview)):
        if not value:
            return None
        return int(value[0])
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def covariance_summary(values) -> dict[str, object]:
    """Summarize an Odometry covariance without streaming all 36 values.

    robot_localization can receive a message with zero covariance for a field
    that is enabled in the EKF configuration.  That is different from the
    intentionally-unused fields in a 6x6 covariance matrix, so report both
    the complete zero set and the planar fields used by the shadow filter.
    This telemetry does not rewrite the message or change EKF input.
    """
    numbers: list[float] = []
    invalid_indices: list[int] = []
    for index, value in enumerate(values):
        try:
            number = float(value)
        except (TypeError, ValueError):
            invalid_indices.append(index)
            continue
        if not math.isfinite(number):
            invalid_indices.append(index)
            continue
        numbers.append(number)
    zero_indices = [
        index for index, value in enumerate(values)
        if index not in invalid_indices and abs(float(value)) <= 1e-12
    ]
    planar_indices = (0, 1, 5)
    planar_zero_indices = [index for index in planar_indices if index in zero_indices]
    if invalid_indices:
        status = "INVALID"
    elif planar_zero_indices:
        status = "ZERO_FUSED_FIELDS"
    elif zero_indices:
        status = "ZERO_UNUSED_FIELDS"
    else:
        status = "OK"
    return {
        "status": status,
        "length": len(values),
        "zero_indices": zero_indices,
        "planar_zero_indices": planar_zero_indices,
        "invalid_indices": invalid_indices,
        "finite_count": len(numbers),
    }


def transform_xy(transform, x: float, y: float) -> list[float]:
    """Apply a planar TransformStamped to an XY point."""
    translation = transform.transform.translation
    yaw = yaw_radians(transform.transform.rotation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        float(translation.x) + cosine * x - sine * y,
        float(translation.y) + sine * x + cosine * y,
    ]


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("real_d435i_odom_live_telemetry")
    qos = QoSProfile(
        depth=50,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    map_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    scan_qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    tf_buffer = tf2_ros.Buffer()
    _tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    latest = {"position": [0.0, 0.0, 0.0], "yaw_deg": 0.0}
    estimator_streams: dict[str, dict[str, object]] = {
        "visual": {
            "topic": "/visual_slam/tracking/odometry",
            "samples": 0,
            "first_stamp_ns": None,
            "last_stamp_ns": None,
            "first_position": None,
            "first_yaw_deg": None,
            "position": [0.0, 0.0, 0.0],
            "yaw_deg": 0.0,
            "frame_id": "",
            "child_frame_id": "",
            "twist": [0.0, 0.0, 0.0],
            "pose_covariance": covariance_summary([0.0] * 36),
            "twist_covariance": covariance_summary([0.0] * 36),
            "gap_samples": deque(maxlen=900),
        },
        "lidar": {
            "topic": "/lidar/odom",
            "samples": 0,
            "first_stamp_ns": None,
            "last_stamp_ns": None,
            "first_position": None,
            "first_yaw_deg": None,
            "position": [0.0, 0.0, 0.0],
            "yaw_deg": 0.0,
            "frame_id": "",
            "child_frame_id": "",
            "twist": [0.0, 0.0, 0.0],
            "pose_covariance": covariance_summary([0.0] * 36),
            "twist_covariance": covariance_summary([0.0] * 36),
            "gap_samples": deque(maxlen=900),
        },
        "ekf_shadow": {
            "topic": "/odometry/filtered_shadow",
            "samples": 0,
            "first_stamp_ns": None,
            "last_stamp_ns": None,
            "first_position": None,
            "first_yaw_deg": None,
            "position": [0.0, 0.0, 0.0],
            "yaw_deg": 0.0,
            "frame_id": "",
            "child_frame_id": "",
            "twist": [0.0, 0.0, 0.0],
            "pose_covariance": covariance_summary([0.0] * 36),
            "twist_covariance": covariance_summary([0.0] * 36),
            "gap_samples": deque(maxlen=900),
        },
    }
    latest_ekf_diagnostics: dict[str, object] = {
        "available": False,
        "name": "",
        "level": None,
        "message": "",
        "values": {},
        "stamp_ns": None,
    }
    latest_odom_guard_diagnostics: dict[str, object] = {
        "available": False,
        "sources": {},
    }
    path: list[list[float]] = []
    latest_map: OccupancyGrid | None = None
    latest_scan: LaserScan | None = None
    latest_localization: dict[str, object] = {
        "available": False,
        "status": "WAIT_AMCL",
        "frame_id": "",
        "stamp_ns": None,
        "x_m": None,
        "y_m": None,
        "yaw_deg": None,
        "covariance_x_m2": None,
        "covariance_y_m2": None,
        "covariance_yaw_rad2": None,
    }
    # slam_toolbox publishes map->odom after the scan/odom timestamp that
    # produced the observation.  Keep a bounded monitor-side queue so the GUI
    # can wait for the complete, same-timestamp map transform instead of
    # clearing the latest scan or composing mixed-time transforms.
    pending_map_poses: deque[tuple[list[float], float, int]] = deque(maxlen=256)
    pending_scans: deque[LaserScan] = deque(maxlen=64)
    pending_map_pose_drops = 0
    pending_scan_drops = 0
    displayed_scan: LaserScan | None = None
    displayed_scan_transform = None
    displayed_scan_delay_ms: float | None = None
    displayed_scan_status = "WAIT"
    MAP_DISPLAY_MAX_WAIT_NS = 3_000_000_000
    SCAN_DISPLAY_MAX_WAIT_NS = 3_000_000_000
    map_frame = "map"
    odom_frame = "odom"
    map_revision = 0
    map_dirty = False
    last_map_signature = ""
    map_path: list[list[float]] = []
    map_robot_pose: list[float] = []
    alignment_map_revision = -1
    alignment_distance_field: list[float] | None = None
    alignment_map_width = 0
    alignment_map_height = 0
    scan_status = "尚未收到 /scan"
    turn_history: list[dict[str, float | int | str]] = []
    status = "—"
    samples = 0
    path_length_m = 0.0
    previous_position: list[float] | None = None
    previous_stamp_ns: int | None = None
    first_odom_stamp_ns: int | None = None
    previous_yaw_deg: float | None = None
    run_start_yaw_deg: float | None = None
    run_yaw_unwrapped_deg = 0.0
    turn_yaw_rate_samples: deque[float] = deque(maxlen=900)
    turn_sign_flips = 0
    previous_turn_sign = 0
    turn_active = False
    turn_quiet_since_ns: int | None = None
    turn_start_position: list[float] | None = None
    turn_start_yaw_deg: float | None = None
    turn_start_stamp_ns: int | None = None
    turn_last_yaw_deg: float | None = None
    turn_last_stamp_ns: int | None = None
    turn_last_position: list[float] | None = None
    turn_max_xy_displacement_m = 0.0
    turn_path_length_m = 0.0
    turn_segments = 0
    last_turn_translation: dict[str, float | int | str] = {
        "status": "未偵測",
        "candidate_status": "未偵測",
        "turn_segments": 0,
        "yaw_delta_deg": 0.0,
        "max_xy_displacement_m": 0.0,
        "path_length_m": 0.0,
    }
    last_gap_ms = 0.0
    gap_samples: deque[float] = deque(maxlen=900)
    vo_status_samples = 0
    vo_valid_samples = 0
    last_emit = 0.0

    def update_estimator_stream(name: str, message: Odometry) -> None:
        stream = estimator_streams[name]
        current_stamp_ns = stamp_nanoseconds(message.header)
        last_stamp_ns = stream["last_stamp_ns"]
        if isinstance(last_stamp_ns, int) and current_stamp_ns <= last_stamp_ns:
            return
        pose = message.pose.pose
        position = [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ]
        yaw_deg = yaw_degrees(pose.orientation)
        if isinstance(last_stamp_ns, int):
            gap_samples = stream["gap_samples"]
            assert isinstance(gap_samples, deque)
            gap_samples.append((current_stamp_ns - last_stamp_ns) / 1e6)
        if stream["first_stamp_ns"] is None:
            stream["first_stamp_ns"] = current_stamp_ns
            stream["first_position"] = list(position)
            stream["first_yaw_deg"] = yaw_deg
        stream["last_stamp_ns"] = current_stamp_ns
        stream["timestamp_age_ms"] = (
            node.get_clock().now().nanoseconds - current_stamp_ns
        ) / 1e6
        stream["position"] = position
        stream["yaw_deg"] = yaw_deg
        stream["frame_id"] = str(message.header.frame_id or "")
        stream["child_frame_id"] = str(message.child_frame_id or "")
        stream["twist"] = [
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
            float(message.twist.twist.angular.z),
        ]
        stream["pose_covariance"] = covariance_summary(message.pose.covariance)
        stream["twist_covariance"] = covariance_summary(message.twist.covariance)
        stream["samples"] = int(stream["samples"]) + 1

    def estimator_stream_snapshot(name: str) -> dict[str, object]:
        stream = estimator_streams[name]
        samples = int(stream["samples"])
        first_stamp_ns = stream["first_stamp_ns"]
        last_stamp_ns = stream["last_stamp_ns"]
        span_s = (
            (int(last_stamp_ns) - int(first_stamp_ns)) / 1e9
            if isinstance(first_stamp_ns, int)
            and isinstance(last_stamp_ns, int)
            and last_stamp_ns > first_stamp_ns
            else 0.0
        )
        gap_samples = stream["gap_samples"]
        assert isinstance(gap_samples, deque)
        position = list(stream["position"])
        first_position = stream["first_position"]
        first_yaw_deg = stream["first_yaw_deg"]
        if not isinstance(first_position, list):
            first_position = [0.0, 0.0, 0.0]
        relative_position = [
            float(position[index]) - float(first_position[index])
            for index in range(3)
        ]
        relative_yaw_deg = (
            (float(stream["yaw_deg"]) - float(first_yaw_deg) + 180.0) % 360.0 - 180.0
            if isinstance(first_yaw_deg, (int, float))
            else 0.0
        )
        return {
            "topic": stream["topic"],
            "status": "WAIT" if samples == 0 else ("STARTING" if samples < 3 else "OK"),
            "samples": samples,
            "rate_hz": ((samples - 1) / span_s if samples > 1 and span_s > 0.0 else 0.0),
            "gap_p95_ms": percentile(gap_samples, 0.95),
            "stamp_ns": last_stamp_ns,
            "timestamp_age_ms": round(float(stream.get("timestamp_age_ms", 0.0)), 3),
            "frame_id": stream["frame_id"],
            "child_frame_id": stream["child_frame_id"],
            "position": position,
            "yaw_deg": float(stream["yaw_deg"]),
            "twist": list(stream["twist"]),
            "covariance": {
                "pose": dict(stream["pose_covariance"]),
                "twist": dict(stream["twist_covariance"]),
                "fused_field_warning": (
                    stream["pose_covariance"].get("status") == "ZERO_FUSED_FIELDS"
                    or stream["twist_covariance"].get("status") == "ZERO_FUSED_FIELDS"
                ),
            },
            "relative_position": relative_position,
            "relative_yaw_deg": relative_yaw_deg,
        }

    def estimator_comparison(sources: dict[str, dict[str, object]]) -> dict[str, dict[str, object]]:
        comparisons: dict[str, dict[str, object]] = {}
        for first, second in (("visual", "lidar"), ("visual", "ekf_shadow"), ("lidar", "ekf_shadow")):
            key = f"{first}_vs_{second}"
            left = sources[first]
            right = sources[second]
            if left["status"] != "OK" or right["status"] != "OK":
                comparisons[key] = {
                    "status": "WAIT",
                    "gate": "PENDING",
                    "source_a": first,
                    "source_b": second,
                    "reason": "兩個 odom source 都需要至少 3 筆有效訊息",
                    "basis": "各 source 自首次收到資料起算；診斷比較，不改寫正式 TF",
                }
                continue
            frame_mismatch = (
                left["frame_id"] != right["frame_id"]
                or left["child_frame_id"] != right["child_frame_id"]
            )
            left_delta = left["relative_position"]
            right_delta = right["relative_position"]
            delta_xy_m = math.hypot(
                float(left_delta[0]) - float(right_delta[0]),
                float(left_delta[1]) - float(right_delta[1]),
            )
            delta_yaw_deg = abs(
                (float(left["relative_yaw_deg"]) - float(right["relative_yaw_deg"]) + 180.0)
                % 360.0 - 180.0
            )
            stamps = [
                int(value)
                for value in (left.get("stamp_ns"), right.get("stamp_ns"))
                if isinstance(value, int)
            ]
            timestamp_skew_ms = (
                (max(stamps) - min(stamps)) / 1e6 if len(stamps) == 2 else 0.0
            )
            gate = (
                "PASS" if delta_xy_m <= 0.10 and delta_yaw_deg <= 5.0
                else ("REVIEW" if delta_xy_m <= 0.20 and delta_yaw_deg <= 10.0 else "FAIL")
            )
            comparisons[key] = {
                "status": "REVIEW" if frame_mismatch or timestamp_skew_ms > 100.0 else "OK",
                "gate": "REVIEW" if (frame_mismatch or timestamp_skew_ms > 100.0) and gate == "PASS" else gate,
                "source_a": first,
                "source_b": second,
                "delta_xy_m": delta_xy_m,
                "delta_yaw_deg": delta_yaw_deg,
                "timestamp_skew_ms": timestamp_skew_ms,
                "frame_mismatch": frame_mismatch,
                "frame_a": left["frame_id"],
                "frame_b": right["frame_id"],
                "child_frame_a": left["child_frame_id"],
                "child_frame_b": right["child_frame_id"],
                "basis": "各 source 自首次收到資料起算；診斷比較，不改寫正式 TF",
            }
        return comparisons

    def projection_estimate(target: list[float], visual: list[float], lidar: list[float]) -> dict[str, object]:
        """Estimate where EKF lies between visual and LiDAR trajectories.

        This is a geometric projection, not the internal Kalman gain.  Values
        outside the input segment are retained as raw_projection and clipped
        only for the operator-facing percentage.
        """
        baseline = [float(lidar[index]) - float(visual[index]) for index in range(len(visual))]
        offset = [float(target[index]) - float(visual[index]) for index in range(len(visual))]
        denominator = sum(value * value for value in baseline)
        if denominator <= 1e-9:
            return {
                "status": "UNDEFINED",
                "cuvslam_pct": None,
                "lidar_pct": None,
                "raw_projection": None,
            }
        raw_projection = sum(offset[index] * baseline[index] for index in range(len(baseline))) / denominator
        clipped_projection = max(0.0, min(1.0, raw_projection))
        return {
            "status": "OK",
            "cuvslam_pct": round(100.0 * (1.0 - clipped_projection), 1),
            "lidar_pct": round(100.0 * clipped_projection, 1),
            "raw_projection": raw_projection,
        }

    def wrapped_delta_degrees(value: float) -> float:
        return (value + 180.0) % 360.0 - 180.0

    def estimator_contribution(sources: dict[str, dict[str, object]]) -> dict[str, object]:
        visual = sources["visual"]
        lidar = sources["lidar"]
        ekf = sources["ekf_shadow"]
        if any(source["status"] != "OK" for source in (visual, lidar, ekf)):
            return {
                "status": "WAIT",
                "basis": "各 source 自首次收到資料起算；診斷比較，不改寫正式 TF",
                "reason": "visual、LiDAR、EKF shadow 都需要至少 3 筆有效訊息",
                "pull_estimate": {},
            }
        stamps = [int(source["stamp_ns"]) for source in (visual, lidar, ekf) if isinstance(source.get("stamp_ns"), int)]
        timestamp_skew_ms = (max(stamps) - min(stamps)) / 1e6 if len(stamps) == 3 else 0.0
        if timestamp_skew_ms > 100.0:
            return {
                "status": "REVIEW",
                "basis": "EKF 相對於 cuVSLAM/LiDAR 軌跡的幾何投影；不是實際 Kalman gain",
                "timestamp_skew_ms": timestamp_skew_ms,
                "input_consistency_gate": "REVIEW",
                "reason": "三個 source 時間差 >100 ms，介入比例不判定",
                "pull_estimate": {
                    "xy": {
                        "status": "UNAVAILABLE",
                        "cuvslam_pct": None,
                        "lidar_pct": None,
                        "raw_projection": None,
                    },
                    "yaw": {
                        "status": "UNAVAILABLE",
                        "cuvslam_pct": None,
                        "lidar_pct": None,
                        "raw_projection": None,
                    },
                },
            }
        visual_position = list(visual["relative_position"])
        lidar_position = list(lidar["relative_position"])
        ekf_position = list(ekf["relative_position"])
        xy_estimate = projection_estimate(
            ekf_position[:2], visual_position[:2], lidar_position[:2]
        )
        visual_yaw = float(visual["relative_yaw_deg"])
        lidar_yaw = float(lidar["relative_yaw_deg"])
        ekf_yaw = float(ekf["relative_yaw_deg"])
        yaw_baseline = wrapped_delta_degrees(lidar_yaw - visual_yaw)
        yaw_offset = wrapped_delta_degrees(ekf_yaw - visual_yaw)
        if abs(yaw_baseline) <= 1e-6:
            yaw_estimate = {
                "status": "UNDEFINED",
                "cuvslam_pct": None,
                "lidar_pct": None,
                "raw_projection": None,
            }
        else:
            raw_projection = yaw_offset / yaw_baseline
            clipped_projection = max(0.0, min(1.0, raw_projection))
            yaw_estimate = {
                "status": "OK",
                "cuvslam_pct": round(100.0 * (1.0 - clipped_projection), 1),
                "lidar_pct": round(100.0 * clipped_projection, 1),
                "raw_projection": raw_projection,
            }
        return {
            "status": "REVIEW" if timestamp_skew_ms > 100.0 else "OK",
            "basis": "EKF 相對於 cuVSLAM/LiDAR 軌跡的幾何投影；不是實際 Kalman gain",
            "timestamp_skew_ms": timestamp_skew_ms,
            "input_consistency_gate": sources.get("visual", {}).get("status", "WAIT"),
            "pull_estimate": {"xy": xy_estimate, "yaw": yaw_estimate},
        }

    def map_pose_for_sample(position: list[float], yaw_deg: float, stamp_ns: int) -> list[float] | None:
        """Project one odom sample with the TF valid at that sample timestamp."""
        if not map_frame or not odom_frame or map_frame == odom_frame:
            return [position[0], position[1], yaw_deg]
        try:
            transform = tf_buffer.lookup_transform(
                map_frame,
                odom_frame,
                Time(nanoseconds=int(stamp_ns)),
                timeout=Duration(seconds=0.0),
            )
        except tf2_ros.TransformException:
            return None
        projected = transform_xy(transform, position[0], position[1])
        projected_yaw = math.degrees(yaw_radians(transform.transform.rotation)) + yaw_deg
        return [projected[0], projected[1], projected_yaw]

    def resolve_pending_map_poses(max_items: int = 64) -> None:
        """Resolve queued odom samples only with exact timestamped map TF.

        This is deliberately monitor-side.  It does not publish TF, change
        the odom source, or reproject old points with the latest correction.
        """
        nonlocal map_path, map_robot_pose, pending_map_pose_drops
        resolved = 0
        now_ns = int(node.get_clock().now().nanoseconds)
        while pending_map_poses and resolved < max_items:
            position, yaw_deg, stamp_ns = pending_map_poses[0]
            map_pose = map_pose_for_sample(position, yaw_deg, stamp_ns)
            if map_pose is None:
                if stamp_ns and now_ns - stamp_ns > MAP_DISPLAY_MAX_WAIT_NS:
                    pending_map_poses.popleft()
                    pending_map_pose_drops += 1
                    continue
                break
            pending_map_poses.popleft()
            map_path.append([map_pose[0], map_pose[1]])
            map_robot_pose = map_pose
            resolved += 1

    def odom_callback(message) -> None:
        nonlocal samples, path_length_m, previous_position, previous_stamp_ns
        nonlocal first_odom_stamp_ns, last_gap_ms, previous_yaw_deg
        nonlocal turn_sign_flips, previous_turn_sign
        nonlocal turn_active, turn_quiet_since_ns, turn_start_position
        nonlocal turn_start_yaw_deg, turn_last_yaw_deg, turn_last_position
        nonlocal turn_start_stamp_ns, turn_last_stamp_ns
        nonlocal turn_max_xy_displacement_m, turn_path_length_m
        nonlocal turn_segments, last_turn_translation
        nonlocal run_start_yaw_deg, run_yaw_unwrapped_deg, odom_frame
        nonlocal pending_map_poses
        odom_frame = str(message.header.frame_id or "odom")
        pose = message.pose.pose
        current_position = [float(pose.position.x), float(pose.position.y), float(pose.position.z)]
        current_stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        current_yaw_deg = yaw_degrees(pose.orientation)
        # A ROS graph can deliver duplicate/stale samples when more than one
        # publisher or a transient reconnection is present.  Do not let those
        # callbacks inflate the displayed path length or odom rate.
        if previous_stamp_ns is not None and current_stamp_ns <= previous_stamp_ns:
            return
        if first_odom_stamp_ns is None:
            first_odom_stamp_ns = current_stamp_ns
        if run_start_yaw_deg is None:
            run_start_yaw_deg = current_yaw_deg
        if previous_position is not None:
            path_length_m += math.sqrt(sum((a - b) ** 2 for a, b in zip(previous_position, current_position)))
        yaw_rate_deg_s = 0.0
        if previous_stamp_ns is not None and current_stamp_ns > previous_stamp_ns:
            last_gap_ms = (current_stamp_ns - previous_stamp_ns) / 1e6
            gap_samples.append(last_gap_ms)
            if previous_yaw_deg is not None:
                yaw_delta_deg = (current_yaw_deg - previous_yaw_deg + 180.0) % 360.0 - 180.0
                run_yaw_unwrapped_deg += yaw_delta_deg
                yaw_rate_deg_s = yaw_delta_deg / max((current_stamp_ns - previous_stamp_ns) / 1e9, 1e-6)
                # This is a diagnostic, not a pass/fail gate.  Small-rate
                # changes are ignored so normal visual noise does not look
                # like a turn; alternating signs during an active turn are
                # retained as a useful jitter indicator.
                if abs(yaw_rate_deg_s) >= 2.0:
                    turn_yaw_rate_samples.append(yaw_rate_deg_s)
                    turn_sign = 1 if yaw_rate_deg_s > 0.0 else -1
                    if previous_turn_sign and turn_sign != previous_turn_sign:
                        turn_sign_flips += 1
                    previous_turn_sign = turn_sign

        # A stable vo_state and a low sign-flip count do not prove that a
        # nominally in-place turn has zero translation.  Track the XY motion
        # during each continuous yaw segment separately so false translation
        # is visible instead of being hidden by the general health score.
        turn_threshold_deg_s = 2.0
        turn_timeout_ns = 250_000_000
        turn_is_active = abs(yaw_rate_deg_s) >= turn_threshold_deg_s
        if turn_is_active:
            if not turn_active:
                turn_active = True
                turn_quiet_since_ns = None
                turn_start_position = list(current_position)
                turn_start_yaw_deg = current_yaw_deg
                turn_start_stamp_ns = current_stamp_ns
                turn_last_yaw_deg = current_yaw_deg
                turn_last_stamp_ns = current_stamp_ns
                turn_last_position = list(current_position)
                turn_max_xy_displacement_m = 0.0
                turn_path_length_m = 0.0
            elif turn_last_position is not None:
                turn_path_length_m += math.sqrt(sum(
                    (a - b) ** 2 for a, b in zip(turn_last_position, current_position)
                ))
                turn_last_position = list(current_position)
                turn_last_yaw_deg = current_yaw_deg
                turn_last_stamp_ns = current_stamp_ns
            if turn_start_position is not None:
                turn_max_xy_displacement_m = max(
                    turn_max_xy_displacement_m,
                    math.hypot(
                        current_position[0] - turn_start_position[0],
                        current_position[1] - turn_start_position[1],
                    ),
                )
            turn_quiet_since_ns = None
        elif turn_active:
            if turn_quiet_since_ns is None:
                turn_quiet_since_ns = current_stamp_ns
            elif current_stamp_ns - turn_quiet_since_ns >= turn_timeout_ns:
                turn_active = False
                turn_segments += 1
                yaw_delta = 0.0
                if turn_start_yaw_deg is not None and turn_last_yaw_deg is not None:
                    yaw_delta = abs((turn_last_yaw_deg - turn_start_yaw_deg + 180.0) % 360.0 - 180.0)
                if yaw_delta >= 30.0:
                    if turn_max_xy_displacement_m <= 0.10:
                        candidate_status = "PASS"
                    elif turn_max_xy_displacement_m <= 0.20:
                        candidate_status = "REVIEW"
                    else:
                        candidate_status = "FAIL"
                    # A yaw segment may be a real curved translation.  Keep
                    # the geometric observation separate from the in-place
                    # turn gate; the latter needs an action label from the
                    # experimenter before it becomes a pass/fail result.
                    turn_status = "待確認"
                else:
                    turn_status = "未偵測"
                    candidate_status = "未偵測"
                last_turn_translation = {
                    "status": turn_status,
                    "candidate_status": candidate_status,
                    "turn_segments": turn_segments,
                    "yaw_delta_deg": yaw_delta,
                    "max_xy_displacement_m": turn_max_xy_displacement_m,
                    "path_length_m": turn_path_length_m,
                }
                turn_history.append({
                    **last_turn_translation,
                    "segment": turn_segments,
                    "start_stamp_ns": turn_start_stamp_ns or 0,
                    "end_stamp_ns": turn_last_stamp_ns or current_stamp_ns,
                })
                turn_quiet_since_ns = None
                turn_start_position = None
                turn_start_yaw_deg = None
                turn_start_stamp_ns = None
                turn_last_yaw_deg = None
                turn_last_stamp_ns = None
                turn_last_position = None
                turn_max_xy_displacement_m = 0.0
                turn_path_length_m = 0.0
        latest["position"] = current_position
        latest["yaw_deg"] = current_yaw_deg
        update_estimator_stream("visual", message)
        path.append([latest["position"][0], latest["position"][1]])
        if pending_map_poses.maxlen and len(pending_map_poses) >= pending_map_poses.maxlen:
            pending_map_poses.popleft()
        pending_map_poses.append((list(current_position), current_yaw_deg, current_stamp_ns))
        samples += 1
        previous_position = current_position
        previous_stamp_ns = current_stamp_ns
        previous_yaw_deg = current_yaw_deg

    def map_callback(message: OccupancyGrid) -> None:
        nonlocal latest_map, map_frame, map_revision, map_dirty, alignment_map_revision
        latest_map = message
        map_frame = str(message.header.frame_id or "map")
        map_revision += 1
        map_dirty = True
        # The scan-map distance field is tied to one OccupancyGrid revision.
        # Force a rebuild when slam_toolbox grows or updates the map.
        alignment_map_revision = -1

    def scan_callback(message: LaserScan) -> None:
        nonlocal latest_scan, pending_scan_drops
        latest_scan = message
        if pending_scans.maxlen and len(pending_scans) >= pending_scans.maxlen:
            pending_scans.popleft()
            pending_scan_drops += 1
        pending_scans.append(message)

    def resolve_pending_scans(max_items: int = 8) -> None:
        """Resolve scans after map TF catches up, retaining the last valid one."""
        nonlocal displayed_scan, displayed_scan_transform
        nonlocal displayed_scan_delay_ms, displayed_scan_status
        nonlocal pending_scan_drops
        resolved = 0
        now_ns = int(node.get_clock().now().nanoseconds)
        while pending_scans and resolved < max_items:
            message = pending_scans[0]
            source_frame = str(message.header.frame_id or "")
            stamp_ns = stamp_nanoseconds(message.header)
            if not source_frame:
                pending_scans.popleft()
                pending_scan_drops += 1
                displayed_scan_status = "SCAN_FRAME_MISSING"
                continue
            try:
                transform = tf_buffer.lookup_transform(
                    map_frame,
                    source_frame,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=0.0),
                )
            except tf2_ros.TransformException:
                if stamp_ns and now_ns - stamp_ns > SCAN_DISPLAY_MAX_WAIT_NS:
                    pending_scans.popleft()
                    pending_scan_drops += 1
                    continue
                break
            pending_scans.popleft()
            displayed_scan = message
            displayed_scan_transform = transform
            displayed_scan_delay_ms = (
                (now_ns - stamp_ns) / 1e6 if stamp_ns else None
            )
            displayed_scan_status = "OK_DELAYED"
            resolved += 1

    def lidar_odom_callback(message: Odometry) -> None:
        update_estimator_stream("lidar", message)

    def ekf_shadow_callback(message: Odometry) -> None:
        update_estimator_stream("ekf_shadow", message)

    def ekf_diagnostics_callback(message: DiagnosticArray) -> None:
        nonlocal latest_ekf_diagnostics
        for diagnostic in message.status:
            name = str(diagnostic.name or "")
            lowered = name.lower()
            if "ekf" not in lowered and "robot_localization" not in lowered:
                continue
            latest_ekf_diagnostics = {
                "available": True,
                "name": name,
                "level": diagnostic_level_value(getattr(diagnostic, "level", None)),
                "message": str(diagnostic.message or ""),
                "values": {
                    str(value.key): str(value.value)
                    for value in diagnostic.values
                },
                "stamp_ns": int(node.get_clock().now().nanoseconds),
            }
            break

    def odom_guard_diagnostics_callback(message: DiagnosticArray) -> None:
        nonlocal latest_odom_guard_diagnostics
        sources = dict(latest_odom_guard_diagnostics.get("sources", {}))
        for diagnostic in message.status:
            name = str(diagnostic.name or "")
            if "odom_contract_guard/" not in name:
                continue
            source = name.split("/", 1)[-1]
            sources[source] = {
                "name": name,
                "level": diagnostic_level_value(getattr(diagnostic, "level", None)),
                "message": str(diagnostic.message or ""),
                "values": {
                    str(value.key): str(value.value)
                    for value in diagnostic.values
                },
                "stamp_ns": int(node.get_clock().now().nanoseconds),
            }
        if sources:
            latest_odom_guard_diagnostics = {
                "available": True,
                "sources": sources,
            }

    def current_map_transform():
        """Use the latest map->odom transform for live visualization only."""
        if not map_frame or not odom_frame or map_frame == odom_frame:
            return None
        return tf_buffer.lookup_transform(
            map_frame,
            odom_frame,
            Time(),
            timeout=Duration(seconds=0.05),
        )

    def build_map_path() -> tuple[list[list[float]], list[float]]:
        # map_path is built incrementally in odom_callback with the TF at each
        # odometry timestamp.  Reprojecting the complete history with the
        # latest map->odom transform makes old points jump whenever
        # slam_toolbox updates its correction.
        return [list(point) for point in map_path], list(map_robot_pose)

    def build_map_payload() -> dict[str, object] | None:
        nonlocal map_dirty, last_map_signature
        if latest_map is None or not map_dirty:
            return None
        message = latest_map
        encoded_cells = bytes(
            255 if int(value) < 0 else max(0, min(100, int(value)))
            for value in message.data
        )
        metadata = (
            f"{message.info.width}:{message.info.height}:"
            f"{message.info.resolution}:{message.info.origin.position.x}:"
            f"{message.info.origin.position.y}:"
            f"{yaw_degrees(message.info.origin.orientation)}:{map_frame}"
        ).encode("utf-8")
        signature = hashlib.sha1(metadata + encoded_cells).hexdigest()
        map_dirty = False
        if signature == last_map_signature:
            return None
        last_map_signature = signature
        return {
            "schema": 1,
            "map": {
                "frame_id": map_frame,
                "revision": map_revision,
                "width": int(message.info.width),
                "height": int(message.info.height),
                "resolution": float(message.info.resolution),
                "origin": [
                    float(message.info.origin.position.x),
                    float(message.info.origin.position.y),
                    float(yaw_degrees(message.info.origin.orientation)),
                ],
                "data_encoding": "uint8_occupancy",
                "data_b64": base64.b64encode(encoded_cells).decode("ascii"),
                "stamp_ns": stamp_nanoseconds(message.header),
            },
        }

    def ensure_alignment_distance_field() -> list[float] | None:
        """Build a small occupancy distance field for the current map revision."""
        nonlocal alignment_map_revision, alignment_distance_field
        nonlocal alignment_map_width, alignment_map_height
        if latest_map is None:
            return None
        width = int(latest_map.info.width)
        height = int(latest_map.info.height)
        if width <= 0 or height <= 0 or len(latest_map.data) != width * height:
            return None
        if alignment_map_revision == map_revision and alignment_distance_field is not None:
            return alignment_distance_field

        field = [0.0 if int(value) >= 65 else math.inf for value in latest_map.data]
        diagonal = math.sqrt(2.0)
        for row in range(height):
            base = row * width
            for column in range(width):
                index = base + column
                best = field[index]
                if column > 0:
                    best = min(best, field[index - 1] + 1.0)
                if row > 0:
                    best = min(best, field[index - width] + 1.0)
                    if column > 0:
                        best = min(best, field[index - width - 1] + diagonal)
                    if column + 1 < width:
                        best = min(best, field[index - width + 1] + diagonal)
                field[index] = best
        for row in range(height - 1, -1, -1):
            base = row * width
            for column in range(width - 1, -1, -1):
                index = base + column
                best = field[index]
                if column + 1 < width:
                    best = min(best, field[index + 1] + 1.0)
                if row + 1 < height:
                    best = min(best, field[index + width] + 1.0)
                    if column > 0:
                        best = min(best, field[index + width - 1] + diagonal)
                    if column + 1 < width:
                        best = min(best, field[index + width + 1] + diagonal)
                field[index] = best
        alignment_map_revision = map_revision
        alignment_distance_field = field
        alignment_map_width = width
        alignment_map_height = height
        return field

    def alignment_distance(x: float, y: float, limit: float) -> float:
        if latest_map is None or alignment_distance_field is None:
            return math.inf
        info = latest_map.info
        origin_yaw = yaw_radians(info.origin.orientation)
        dx = x - float(info.origin.position.x)
        dy = y - float(info.origin.position.y)
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        column = int(math.floor(local_x / float(info.resolution)))
        row = int(math.floor(local_y / float(info.resolution)))
        if (
            column < 0 or column >= alignment_map_width
            or row < 0 or row >= alignment_map_height
        ):
            return math.inf
        distance = alignment_distance_field[row * alignment_map_width + column] * float(info.resolution)
        return distance if distance <= limit + float(info.resolution) else math.inf

    def scan_map_alignment(
        scan: LaserScan,
        transform,
        raw_hits: list[tuple[float, float]],
        measured_count: int,
        near_count: int,
        far_count: int,
    ) -> dict[str, object]:
        field = ensure_alignment_distance_field()
        if field is None or measured_count == 0:
            return {"status": "WAIT", "reason": "map or valid scan unavailable"}
        near_ratio = near_count / measured_count
        far_ratio = far_count / measured_count
        candidates: list[tuple[float, str]] = []
        for angle_sign, sign_name in ((1.0, "normal"), (-1.0, "mirrored")):
            for offset, offset_name in (
                (0.0, "0deg"),
                (math.pi / 2.0, "+90deg"),
                (math.pi, "180deg"),
                (-math.pi / 2.0, "-90deg"),
            ):
                hits = 0
                candidate_hits = raw_hits[::2]
                for distance, angle in candidate_hits:
                    candidate_angle = angle_sign * angle + offset
                    point = transform_xy(
                        transform,
                        distance * math.cos(candidate_angle),
                        distance * math.sin(candidate_angle),
                    )
                    if alignment_distance(point[0], point[1], 0.30) <= 0.30:
                        hits += 1
                candidates.append((hits / max(1, len(candidate_hits)), f"{sign_name}/{offset_name}"))
        best_ratio, best_orientation = max(candidates)
        return {
            "status": "MEASURED",
            "near_ratio": round(near_ratio, 4),
            "far_ratio": round(far_ratio, 4),
            "best_ratio": round(best_ratio, 4),
            "best_orientation": best_orientation,
            "measured_points": measured_count,
            "map_revision": map_revision,
            "stamp_ns": stamp_nanoseconds(scan.header),
        }

    def amcl_pose_callback(message: PoseWithCovarianceStamped) -> None:
        nonlocal latest_localization
        pose = message.pose.pose
        covariance = message.pose.covariance
        latest_localization = {
            "available": True,
            "status": "OK",
            "frame_id": str(message.header.frame_id or "map"),
            "stamp_ns": stamp_nanoseconds(message.header),
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "yaw_deg": float(yaw_degrees(pose.orientation)),
            "covariance_x_m2": float(covariance[0]) if len(covariance) > 0 else None,
            "covariance_y_m2": float(covariance[7]) if len(covariance) > 7 else None,
            "covariance_yaw_rad2": float(covariance[35]) if len(covariance) > 35 else None,
        }

    def scan_timing_payload(
        scan: LaserScan,
        *,
        tf_lookup_status: str = "NOT_REQUESTED",
        transform=None,
        latest_transform=None,
        tf_error: str = "",
        display_mode: str = "not_displayed",
        display_delay_ms: float | None = None,
        queue_depth: int = 0,
        queue_drops: int = 0,
    ) -> dict[str, object]:
        """Expose LaserScan timing and the TF timestamp used for observation."""
        scan_stamp_ns = stamp_nanoseconds(scan.header)
        scan_time_s = float(scan.scan_time)
        time_increment_s = float(scan.time_increment)
        ray_count = len(scan.ranges)
        beam_span_s = max(0.0, (ray_count - 1) * time_increment_s) if (
            math.isfinite(time_increment_s) and time_increment_s >= 0.0
        ) else 0.0

        def transform_stamp_ns(value) -> int | None:
            if value is None:
                return None
            try:
                return stamp_nanoseconds(value.header)
            except (AttributeError, TypeError, ValueError):
                return None

        transform_stamp = transform_stamp_ns(transform)
        latest_stamp = transform_stamp_ns(latest_transform)

        def signed_delta_ms(stamp_ns: int | None) -> float | None:
            if stamp_ns is None or stamp_ns == 0 or scan_stamp_ns == 0:
                return None
            return round((stamp_ns - scan_stamp_ns) / 1e6, 3)

        now_stamp_ns = int(node.get_clock().now().nanoseconds)
        scan_end_stamp_ns = scan_stamp_ns + int(beam_span_s * 1e9)
        return {
            "status": "MEASURED" if ray_count > 0 else "WAIT",
            "scan_stamp_ns": scan_stamp_ns,
            "scan_period_s": round(scan_time_s, 9) if math.isfinite(scan_time_s) else None,
            "scan_time_s": round(scan_time_s, 9) if math.isfinite(scan_time_s) else None,
            "time_increment_s": round(time_increment_s, 9) if math.isfinite(time_increment_s) else None,
            "time_increment_us": round(time_increment_s * 1e6, 3) if math.isfinite(time_increment_s) else None,
            "ray_count": ray_count,
            "beam_span_s": round(beam_span_s, 9),
            "beam_span_ms": round(beam_span_s * 1e3, 3),
            "scan_end_stamp_ns": scan_end_stamp_ns,
            "timestamp_age_ms": round((now_stamp_ns - scan_stamp_ns) / 1e6, 3),
            "scan_end_age_ms": round((now_stamp_ns - scan_end_stamp_ns) / 1e6, 3),
            "tf_lookup_status": tf_lookup_status,
            "tf_lookup_requested_stamp_ns": scan_stamp_ns,
            "tf_transform_stamp_ns": transform_stamp,
            "tf_lookup_delta_ms": signed_delta_ms(transform_stamp),
            "tf_vs_scan_end_delta_ms": (
                round((transform_stamp - scan_end_stamp_ns) / 1e6, 3)
                if transform_stamp not in (None, 0) else None
            ),
            "tf_latest_stamp_ns": latest_stamp,
            "tf_latest_delta_ms": signed_delta_ms(latest_stamp),
            "tf_error": tf_error,
            "display_mode": display_mode,
            "display_delay_ms": (
                round(float(display_delay_ms), 3)
                if display_delay_ms is not None else None
            ),
            "display_stamp_ns": scan_stamp_ns if display_mode == "timestamped_delayed" else None,
            "queue_depth": int(queue_depth),
            "queue_drops": int(queue_drops),
        }

    def build_scan_payload() -> dict[str, object]:
        nonlocal scan_status
        if latest_scan is None:
            scan_status = "尚未收到 /scan"
            return {
                "schema": 1, "status": scan_status, "frame_id": "",
                "points": [], "source_points": 0, "display_points": 0,
                "alignment": {"status": "WAIT"},
                "timing": {"status": "WAIT"},
            }
        message = displayed_scan
        if message is None:
            message = latest_scan
            timing = scan_timing_payload(
                message,
                tf_lookup_status="WAITING_FOR_MAP_TF",
                display_mode="waiting_timestamped_map_tf",
                queue_depth=len(pending_scans),
                queue_drops=pending_scan_drops,
            )
            latest_transform = None
            source_frame = str(message.header.frame_id or "")
            if source_frame:
                try:
                    latest_transform = tf_buffer.lookup_transform(
                        map_frame,
                        source_frame,
                        Time(),
                        timeout=Duration(seconds=0.02),
                    )
                except tf2_ros.TransformException:
                    pass
                timing = scan_timing_payload(
                    message,
                    tf_lookup_status="WAITING_FOR_MAP_TF",
                    latest_transform=latest_transform,
                    display_mode="waiting_timestamped_map_tf",
                    queue_depth=len(pending_scans),
                    queue_drops=pending_scan_drops,
                )
            scan_status = "WAITING_FOR_MAP_TF"
            return {
                "schema": 1, "status": scan_status, "frame_id": map_frame,
                "source_frame": source_frame, "points": [],
                "source_points": len(message.ranges), "display_points": 0,
                "alignment": {"status": "WAIT"},
                "stamp_ns": stamp_nanoseconds(message.header),
                "timing": timing,
            }
        source_frame = str(message.header.frame_id or "")
        timing = scan_timing_payload(
            message,
            tf_lookup_status="OK_DELAYED",
            transform=displayed_scan_transform,
            display_mode="timestamped_delayed",
            display_delay_ms=displayed_scan_delay_ms,
            queue_depth=len(pending_scans),
            queue_drops=pending_scan_drops,
        )
        if not source_frame:
            scan_status = "SCAN_FRAME_MISSING"
            return {
                "schema": 1, "status": scan_status, "frame_id": map_frame,
                "points": [], "source_points": len(message.ranges), "display_points": 0,
                "alignment": {"status": "WAIT"},
                "timing": timing,
            }
        transform = displayed_scan_transform
        if transform is None:
            scan_status = displayed_scan_status
            return {
                "schema": 1, "status": scan_status, "frame_id": map_frame,
                "source_frame": source_frame, "points": [],
                "source_points": len(message.ranges), "display_points": 0,
                "alignment": {"status": "WAIT"},
                "stamp_ns": stamp_nanoseconds(message.header),
                "timing": timing,
            }
        step = max(1, math.ceil(len(message.ranges) / 720))
        points: list[list[float]] = []
        raw_hits: list[tuple[float, float]] = []
        near_count = 0
        far_count = 0
        ensure_alignment_distance_field()
        for index in range(0, len(message.ranges), step):
            distance = float(message.ranges[index])
            if not math.isfinite(distance):
                continue
            if distance < float(message.range_min) or distance > float(message.range_max):
                continue
            angle = float(message.angle_min) + index * float(message.angle_increment)
            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)
            point = transform_xy(transform, local_x, local_y)
            points.append(point)
            raw_hits.append((distance, angle))
            if alignment_distance(point[0], point[1], 0.15) <= 0.15:
                near_count += 1
            if alignment_distance(point[0], point[1], 0.30) <= 0.30:
                far_count += 1
        scan_status = "OK_DELAYED" if points else "NO_VALID_POINTS"
        return {
            "schema": 1,
            "status": scan_status,
            "frame_id": map_frame,
            "source_frame": source_frame,
            "points": points,
            "source_points": len(message.ranges),
            "display_points": len(points),
            "stamp_ns": stamp_nanoseconds(message.header),
            "timing": timing,
            "alignment": scan_map_alignment(
                message, transform, raw_hits, len(points), near_count, far_count
            ),
        }

    def status_callback(message) -> None:
        nonlocal status, vo_status_samples, vo_valid_samples
        status = getattr(message, "vo_state", "—")
        if status != "—":
            # Count the status stream itself rather than labelling each odom
            # callback with the most recently received status.  The latter
            # can over/under-count during scheduling jitter and makes the
            # reported VO ratio look more precise than it is.
            vo_status_samples += 1
            if str(status) in {"1", "1.0"}:
                vo_valid_samples += 1

    def percentile(values: deque[float], fraction: float) -> float:
        if not values:
            return 0.0
        ordered = sorted(values)
        index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
        return ordered[index]

    def jitter_summary() -> dict[str, float | int | str]:
        magnitudes = deque(abs(value) for value in turn_yaw_rate_samples)
        turn_samples = len(turn_yaw_rate_samples)
        p95 = percentile(magnitudes, 0.95)
        flip_ratio = turn_sign_flips / max(turn_samples - 1, 1)
        if turn_samples < 10:
            label = "未偵測"
        elif turn_sign_flips >= 4 and flip_ratio >= 0.04:
            label = "REVIEW"
        else:
            label = "OK"
        return {
            "status": label,
            "turn_samples": turn_samples,
            "yaw_rate_p95_deg_s": p95,
            "sign_flips": turn_sign_flips,
            "flip_ratio": flip_ratio,
        }

    def turn_translation_summary() -> dict[str, float | int | str]:
        summary = dict(last_turn_translation)
        if turn_active and turn_start_yaw_deg is not None:
            active_yaw_delta = abs((turn_last_yaw_deg - turn_start_yaw_deg + 180.0) % 360.0 - 180.0)
            if active_yaw_delta >= 30.0:
                active_candidate_status = (
                    "PASS" if turn_max_xy_displacement_m <= 0.10
                    else ("REVIEW" if turn_max_xy_displacement_m <= 0.20 else "FAIL")
                )
            else:
                active_candidate_status = "未偵測"
            summary.update({
                "status": "轉向中",
                "candidate_status": active_candidate_status,
                "yaw_delta_deg": active_yaw_delta if turn_last_yaw_deg is not None else 0.0,
                "max_xy_displacement_m": turn_max_xy_displacement_m,
                "path_length_m": turn_path_length_m,
            })
        return summary

    node.create_subscription(Odometry, "/visual_slam/tracking/odometry", odom_callback, qos)
    node.create_subscription(Odometry, "/lidar/odom", lidar_odom_callback, qos)
    node.create_subscription(Odometry, "/odometry/filtered_shadow", ekf_shadow_callback, qos)
    node.create_subscription(DiagnosticArray, "/diagnostics", ekf_diagnostics_callback, qos)
    node.create_subscription(
        DiagnosticArray,
        "/fusion/odom_contract_diagnostics",
        odom_guard_diagnostics_callback,
        qos,
    )
    node.create_subscription(
        get_message("isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus"),
        "/visual_slam/status",
        status_callback,
        qos,
    )
    node.create_subscription(OccupancyGrid, "/map", map_callback, map_qos)
    node.create_subscription(LaserScan, "/scan", scan_callback, scan_qos)
    node.create_subscription(
        PoseWithCovarianceStamped,
        "/amcl_pose",
        amcl_pose_callback,
        qos,
    )

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        resolve_pending_map_poses()
        resolve_pending_scans()
        now = time.monotonic()
        if now - last_emit < 0.2:
            continue
        last_emit = now
        updated_map = build_map_payload()
        if updated_map is not None:
            print("LIVE_MAP " + json.dumps(updated_map, separators=(",", ":")), flush=True)
        map_path, map_robot_pose = build_map_path()
        # Compute the stream rate from accepted odometry timestamps.  Using
        # collector wall-clock startup time includes ROS discovery/container
        # warm-up before the first odom message and falsely lowers the live
        # rate during the first part of a run.
        timestamp_span_s = (
            (previous_stamp_ns - first_odom_stamp_ns) / 1e9
            if first_odom_stamp_ns is not None and previous_stamp_ns is not None
            else 0.0
        )
        odom_rate_hz = ((samples - 1) / timestamp_span_s
                        if samples > 1 and timestamp_span_s > 0.0 else 0.0)
        estimator_sources = {
            name: estimator_stream_snapshot(name)
            for name in ("visual", "lidar", "ekf_shadow")
        }
        payload = {
            "position": latest["position"],
            "yaw_deg": latest["yaw_deg"],
            "vo_state": status,
            "odom_rate_hz": odom_rate_hz,
            "odom_rate_basis": "ROS odometry timestamp span",
            "odom_timestamp_span_s": timestamp_span_s,
            "turn_jitter": jitter_summary(),
            "turn_translation": turn_translation_summary(),
            "path": path,
            "run_start_yaw_deg": run_start_yaw_deg,
            "run_yaw_delta_deg": (
                abs((latest["yaw_deg"] - run_start_yaw_deg + 180.0) % 360.0 - 180.0)
                if run_start_yaw_deg is not None else 0.0
            ),
            "run_yaw_unwrapped_deg": run_yaw_unwrapped_deg,
            "turn_history": list(turn_history),
            "map_frame": map_frame,
            "map_path": map_path,
            "robot_pose_map": map_robot_pose,
            "map_path_projection": {
                "mode": "timestamped_delayed",
                "queue_depth": len(pending_map_poses),
                "queue_drops": pending_map_pose_drops,
                "max_wait_s": MAP_DISPLAY_MAX_WAIT_NS / 1e9,
            },
            "samples": samples,
            "path_length_m": path_length_m,
            "last_gap_ms": last_gap_ms,
            "gap_p95_ms": percentile(gap_samples, 0.95),
            "vo_valid_ratio": (100.0 * vo_valid_samples / vo_status_samples
                                if vo_status_samples else None),
            "vo_status_samples": vo_status_samples,
            "vo_valid_samples": vo_valid_samples,
            "estimator_sources": estimator_sources,
            "estimator_comparisons": estimator_comparison(estimator_sources),
            "estimator_contribution": estimator_contribution(estimator_sources),
            "ekf_diagnostics": latest_ekf_diagnostics,
            "odom_guard_diagnostics": latest_odom_guard_diagnostics,
            "localization_pose": latest_localization,
        }
        print("LIVE_ODOM " + json.dumps(payload, separators=(",", ":")), flush=True)
        print(
            "LIVE_SCAN " + json.dumps(build_scan_payload(), separators=(",", ":")),
            flush=True,
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
