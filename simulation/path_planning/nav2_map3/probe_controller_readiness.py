#!/usr/bin/env python3
"""No-daemon readiness probe for isolated Foxy Nav2 controller, no motion."""

import json
import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters


def call(node: Node, service_type, name: str, request):
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=5.0):
        raise TimeoutError(f"missing service {name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None:
        raise TimeoutError(f"no reply from {name}")
    return future.result()


def main() -> None:
    rclpy.init()
    node = Node("map3_controller_readiness_probe")
    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.count_publishers("/cmd_vel") == 1
                and node.count_subscribers("/cmd_vel") == 1
                and node.count_publishers("/odom") == 1
                and node.count_publishers("/tf_static") == 1
            ):
                break
        endpoints = {
            topic: {
                "publishers": node.count_publishers(topic),
                "subscribers": node.count_subscribers(topic),
            }
            for topic in ("/cmd_vel", "/odom", "/tf", "/tf_static")
        }
        lifecycle = {
            name: call(node, GetState, f"/{name}/get_state", GetState.Request()).current_state.label
            for name in ("map_server", "planner_server", "controller_server")
        }
        cost_request = GetParameters.Request()
        cost_request.names = ["robot_radius", "track_unknown_space"]
        cost_values = call(
            node, GetParameters, "/local_costmap/local_costmap/get_parameters", cost_request,
        ).values
        controller_request = GetParameters.Request()
        controller_request.names = ["FollowPath.desired_linear_vel", "controller_frequency"]
        controller_values = call(
            node, GetParameters, "/controller_server/get_parameters", controller_request,
        ).values
        evidence = {
            "status": "isolated_nav2_controller_readiness_no_motion",
            "lifecycle": lifecycle,
            "endpoints": endpoints,
            "local_costmap_robot_radius_m": cost_values[0].double_value,
            "local_costmap_track_unknown_space": cost_values[1].bool_value,
            "desired_linear_vel_mps": controller_values[0].double_value,
            "controller_frequency_hz": controller_values[1].double_value,
        }
        if (
            any(value != "active" for value in lifecycle.values())
            or endpoints["/cmd_vel"]["publishers"] != 1
            or endpoints["/cmd_vel"]["subscribers"] != 1
            or endpoints["/odom"]["publishers"] != 1
            or endpoints["/tf_static"]["publishers"] != 1
            or abs(evidence["local_costmap_robot_radius_m"] - 0.61) > 1e-9
            or not evidence["local_costmap_track_unknown_space"]
            or abs(evidence["desired_linear_vel_mps"] - 0.22) > 1e-9
        ):
            raise AssertionError(f"controller readiness gate failed: {evidence}")
        print(json.dumps(evidence, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
