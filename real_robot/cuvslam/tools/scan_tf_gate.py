#!/usr/bin/env python3
"""Release LaserScan messages only after their odom-time TF is available.

The RPLIDAR and cuVSLAM streams can arrive with different processing
latencies.  This gate preserves the original LaserScan message and timestamp,
but waits briefly before publishing it to the mapper.  It does not publish TF
and does not alter odometry ownership.
"""

from __future__ import annotations

import argparse
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros


class ScanTfGate(Node):
    def __init__(self, input_topic: str, output_topic: str, target_frame: str,
                 max_wait_s: float, queue_size: int, check_hz: float) -> None:
        super().__init__("scan_tf_gate")
        self._input_topic = input_topic
        self._output_topic = output_topic
        self._target_frame = target_frame
        self._max_wait_ns = int(max_wait_s * 1e9)
        self._queue_size = max(1, queue_size)
        self._pending: deque[tuple[LaserScan, int]] = deque()
        self._published = 0
        self._dropped = 0
        self._last_wait_ms: float | None = None
        self._last_tf_delta_ms: float | None = None
        self._last_report_ns = 0

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            LaserScan, input_topic, self._scan_callback, qos_profile_sensor_data
        )
        self._timer = self.create_timer(
            1.0 / max(1.0, check_hz), self._flush_queue
        )
        self.get_logger().info(
            "scan_tf_gate input=%s output=%s target=%s max_wait=%.3fs"
            % (input_topic, output_topic, target_frame, max_wait_s)
        )

    @staticmethod
    def _stamp_ns(message: LaserScan) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )

    def _scan_callback(self, message: LaserScan) -> None:
        if not message.header.frame_id or self._stamp_ns(message) <= 0:
            self._dropped += 1
            self.get_logger().warning("drop scan with missing frame or timestamp")
            return
        if len(self._pending) >= self._queue_size:
            self._pending.popleft()
            self._dropped += 1
            self.get_logger().warning(
                "scan_tf_gate queue full; dropping oldest scan pending=%d"
                % len(self._pending)
            )
        self._pending.append((message, self.get_clock().now().nanoseconds))

    def _flush_queue(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        while self._pending:
            message, received_ns = self._pending[0]
            stamp_ns = self._stamp_ns(message)
            try:
                available = self._tf_buffer.can_transform(
                    self._target_frame,
                    message.header.frame_id,
                    Time(nanoseconds=stamp_ns),
                    timeout=Duration(seconds=0.0),
                )
            except tf2_ros.TransformException:
                available = False

            if available:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        self._target_frame,
                        message.header.frame_id,
                        Time(nanoseconds=stamp_ns),
                        timeout=Duration(seconds=0.0),
                    )
                except tf2_ros.TransformException:
                    break
                tf_stamp_ns = (
                    int(transform.header.stamp.sec) * 1_000_000_000
                    + int(transform.header.stamp.nanosec)
                )
                self._pending.popleft()
                self._publisher.publish(message)
                self._published += 1
                self._last_wait_ms = (now_ns - received_ns) / 1e6
                self._last_tf_delta_ms = (tf_stamp_ns - stamp_ns) / 1e6
                continue

            if now_ns - received_ns >= self._max_wait_ns:
                self._pending.popleft()
                self._dropped += 1
                self.get_logger().warning(
                    "drop scan after TF wait %.1f ms frame=%s pending=%d"
                    % ((now_ns - received_ns) / 1e6,
                       message.header.frame_id,
                       len(self._pending))
                )
                continue
            break

        if now_ns - self._last_report_ns < 1_000_000_000:
            return
        self._last_report_ns = now_ns
        self.get_logger().info(
            "scan_tf_gate pending=%d published=%d dropped=%d wait_ms=%s tf_delta_ms=%s"
            % (
                len(self._pending),
                self._published,
                self._dropped,
                "—" if self._last_wait_ms is None else f"{self._last_wait_ms:.1f}",
                "—" if self._last_tf_delta_ms is None else f"{self._last_tf_delta_ms:.1f}",
            )
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="/scan_raw")
    parser.add_argument("--output", default="/scan")
    parser.add_argument("--target-frame", default="odom")
    parser.add_argument("--max-wait-s", type=float, default=2.0)
    parser.add_argument("--queue-size", type=int, default=32)
    parser.add_argument("--check-hz", type=float, default=30.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ScanTfGate(
        args.input,
        args.output,
        args.target_frame,
        args.max_wait_s,
        args.queue_size,
        args.check_hz,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
