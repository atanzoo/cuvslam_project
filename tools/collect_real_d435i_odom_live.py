#!/usr/bin/env python3
"""Stream lightweight real-D435i odometry telemetry for the local GUI."""

from __future__ import annotations

import json
import math
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def yaw_degrees(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("real_d435i_odom_live_telemetry")
    qos = QoSProfile(
        depth=50,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    latest = {"position": [0.0, 0.0, 0.0], "yaw_deg": 0.0}
    path: list[list[float]] = []
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

    def odom_callback(message) -> None:
        nonlocal samples, path_length_m, previous_position, previous_stamp_ns
        nonlocal first_odom_stamp_ns, last_gap_ms, previous_yaw_deg
        nonlocal turn_sign_flips, previous_turn_sign
        nonlocal turn_active, turn_quiet_since_ns, turn_start_position
        nonlocal turn_start_yaw_deg, turn_last_yaw_deg, turn_last_position
        nonlocal turn_start_stamp_ns, turn_last_stamp_ns
        nonlocal turn_max_xy_displacement_m, turn_path_length_m
        nonlocal turn_segments, last_turn_translation
        nonlocal run_start_yaw_deg, run_yaw_unwrapped_deg
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
        path.append([latest["position"][0], latest["position"][1]])
        samples += 1
        previous_position = current_position
        previous_stamp_ns = current_stamp_ns
        previous_yaw_deg = current_yaw_deg

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
    node.create_subscription(
        get_message("isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus"),
        "/visual_slam/status",
        status_callback,
        qos,
    )

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now - last_emit < 0.2:
            continue
        last_emit = now
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
            "samples": samples,
            "path_length_m": path_length_m,
            "last_gap_ms": last_gap_ms,
            "gap_p95_ms": percentile(gap_samples, 0.95),
            "vo_valid_ratio": (100.0 * vo_valid_samples / vo_status_samples
                                if vo_status_samples else None),
            "vo_status_samples": vo_status_samples,
            "vo_valid_samples": vo_valid_samples,
        }
        print("LIVE_ODOM " + json.dumps(payload, separators=(",", ":")), flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
