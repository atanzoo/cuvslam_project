"""Publish Gazebo ground-truth world outlines for Foxglove evaluation."""

from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import Transform, compose, inverse
from .sdf_geometry import BoxGeometry, box_edge_points, load_world


def transform_from_message(message) -> Transform:
    return Transform(
        translation=(
            message.translation.x,
            message.translation.y,
            message.translation.z,
        ),
        rotation=(
            message.rotation.x,
            message.rotation.y,
            message.rotation.z,
            message.rotation.w,
        ),
    )


def pose_from_message(message) -> Transform:
    return Transform(
        translation=(
            message.position.x,
            message.position.y,
            message.position.z,
        ),
        rotation=(
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ),
    )


class WorldOutlinePublisher(Node):
    def __init__(self) -> None:
        super().__init__("world_outline_publisher")
        default_world = (
            get_package_share_directory("cuvslam_sim_observability")
            + "/worlds/indoor_gz_sim.sdf"
        )
        self.declare_parameter("world_sdf", default_world)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter(
            "camera_frame",
            "camera_infra1_optical_frame",
        )
        self.declare_parameter(
            "world_frame",
            "simulation_world_ground_truth",
        )
        self.declare_parameter(
            "marker_topic",
            "/simulation/ground_truth_world_markers",
        )

        world_sdf = str(self.get_parameter("world_sdf").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        self.geometry = load_world(world_sdf)

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            marker_topic,
            marker_qos,
        )
        self.create_subscription(
            PoseStamped,
            "/simulation/native_pose",
            self.native_pose_callback,
            odom_qos,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.latest_ground_truth: PoseStamped | None = None
        self.aligned = False
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(
            f"Loaded {len(self.geometry.boxes)} static box outlines from "
            f"{world_sdf}; waiting for evaluation-only native pose and "
            "cuVSLAM TF."
        )

    def native_pose_callback(self, message: PoseStamped) -> None:
        self.latest_ground_truth = message

    def timer_callback(self) -> None:
        if not self.aligned:
            self.try_align()
            return
        self.publish_markers()

    def try_align(self) -> None:
        if self.latest_ground_truth is None:
            return
        try:
            map_camera_message = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                Time.from_msg(self.latest_ground_truth.header.stamp),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return

        odom_base = pose_from_message(self.latest_ground_truth.pose)
        odom_camera = compose(
            odom_base,
            self.geometry.base_camera_pose,
        )
        map_camera = transform_from_message(
            map_camera_message.transform
        )
        # The native pose is already an absolute Gazebo-world pose.  Applying
        # inverse(initial_robot_pose) here a second time double-counts the
        # initial offset and shifts the SDF outline by about 1.8 m in x/y.
        map_world = compose(map_camera, inverse(odom_camera))

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.world_frame
        transform.transform.translation.x = map_world.translation[0]
        transform.transform.translation.y = map_world.translation[1]
        transform.transform.translation.z = map_world.translation[2]
        transform.transform.rotation.x = map_world.rotation[0]
        transform.transform.rotation.y = map_world.rotation[1]
        transform.transform.rotation.z = map_world.rotation[2]
        transform.transform.rotation.w = map_world.rotation[3]
        self.static_broadcaster.sendTransform(transform)
        self.aligned = True
        self.get_logger().info(
            "Published evaluation-only map -> "
            f"{self.world_frame} alignment."
        )
        self.publish_markers()

    @staticmethod
    def marker_color(box: BoxGeometry) -> tuple[float, float, float, float]:
        if box.model_name == "floor":
            return (0.45, 0.48, 0.52, 0.45)
        if "wall" in box.model_name:
            return (0.10, 0.80, 0.95, 0.95)
        return (1.0, 0.75, 0.10, 1.0)

    def publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        for marker_id, box in enumerate(self.geometry.boxes):
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self.world_frame
            marker.ns = "ground_truth_world_outline"
            marker.id = marker_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.position.x = box.pose.translation[0]
            marker.pose.position.y = box.pose.translation[1]
            marker.pose.position.z = box.pose.translation[2]
            marker.pose.orientation.x = box.pose.rotation[0]
            marker.pose.orientation.y = box.pose.rotation[1]
            marker.pose.orientation.z = box.pose.rotation[2]
            marker.pose.orientation.w = box.pose.rotation[3]
            marker.scale.x = 0.025
            color = self.marker_color(box)
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            marker.points = [
                Point(x=point[0], y=point[1], z=point[2])
                for point in box_edge_points(box.size)
            ]
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldOutlinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
