"""Add the declared stereo baseline to simulated right CameraInfo."""

from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo

from .calibration import projection_with_baseline


class RightCameraInfoAdapter(Node):
    def __init__(self) -> None:
        super().__init__("right_camera_info_adapter")
        self.declare_parameter(
            "input_topic",
            "/d435i/infra2/camera_info",
        )
        self.declare_parameter(
            "output_topic",
            "/cuvslam/input/infra2/camera_info",
        )
        self.declare_parameter("stereo_baseline_m", 0.05)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.baseline_m = float(
            self.get_parameter("stereo_baseline_m").value
        )
        if self.baseline_m <= 0.0:
            raise ValueError("stereo_baseline_m must be positive")

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            CameraInfo,
            output_topic,
            qos,
        )
        self.create_subscription(
            CameraInfo,
            input_topic,
            self.camera_info_callback,
            qos,
        )
        self.logged_correction = False
        self.logged_error = False
        self.get_logger().info(
            f"Normalizing right CameraInfo from {input_topic} to "
            f"{output_topic}; baseline={self.baseline_m:.6f}m."
        )

    def camera_info_callback(self, message: CameraInfo) -> None:
        try:
            corrected_projection = projection_with_baseline(
                message.p,
                self.baseline_m,
            )
        except ValueError as exc:
            if not self.logged_error:
                self.get_logger().error(str(exc))
                self.logged_error = True
            return

        corrected = deepcopy(message)
        corrected.p = corrected_projection
        self.publisher.publish(corrected)
        if not self.logged_correction:
            self.get_logger().info(
                "Published corrected right CameraInfo: "
                f"P[0]={corrected.p[0]:.6f}, "
                f"P[3]={corrected.p[3]:.6f}."
            )
            self.logged_correction = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RightCameraInfoAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
