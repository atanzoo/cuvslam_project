#!/usr/bin/env python3
"""Read-only readiness gate for isolated Humble Nav2 MPPI; sends no goal."""

from __future__ import annotations

import json
import math
import time
import argparse
from pathlib import Path

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowPath
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node


def call(node: Node, service_type, name: str, request):
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=5.0):
        raise TimeoutError(f"missing service {name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None:
        raise TimeoutError(f"no reply from {name}")
    return future.result()


def parameter_value(value):
    if value.type == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    if value.type == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if value.type == ParameterType.PARAMETER_STRING:
        return value.string_value
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--expected-vx-max", type=float, default=0.22)
    parser.add_argument("--expected-wz-max", type=float, default=0.60)
    args = parser.parse_args()

    rclpy.init()
    node = Node("map3_mppi_readiness_probe")
    try:
        def endpoint_snapshot():
            return {
                topic: {
                    "publishers": node.count_publishers(topic),
                    "subscribers": node.count_subscribers(topic),
                }
                for topic in ("/cmd_vel", "/odom", "/tf", "/tf_static")
            }

        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            current_endpoints = endpoint_snapshot()
            if (
                current_endpoints["/cmd_vel"] == {"publishers": 1, "subscribers": 1}
                and current_endpoints["/odom"]["publishers"] == 1
                and current_endpoints["/tf_static"]["publishers"] == 1
            ):
                break
        endpoints = endpoint_snapshot()
        lifecycle = {
            name: call(node, GetState, f"/{name}/get_state", GetState.Request())
            .current_state.label
            for name in ("map_server", "planner_server", "controller_server")
        }

        controller_request = GetParameters.Request()
        controller_request.names = [
            "FollowPath.vx_max", "FollowPath.wz_max", "FollowPath.time_steps",
            "FollowPath.model_dt", "FollowPath.motion_model", "controller_frequency",
        ]
        controller_values = call(
            node, GetParameters, "/controller_server/get_parameters", controller_request,
        ).values
        costmap_request = GetParameters.Request()
        costmap_request.names = ["robot_radius", "track_unknown_space"]
        costmap_values = call(
            node, GetParameters,
            "/local_costmap/local_costmap/get_parameters", costmap_request,
        ).values
        action_ready = ActionClient(node, FollowPath, "/follow_path").wait_for_server(
            timeout_sec=3.0,
        )
        parameters = {
            name: parameter_value(value)
            for name, value in zip(controller_request.names, controller_values)
        }
        costmap = {
            name: parameter_value(value)
            for name, value in zip(costmap_request.names, costmap_values)
        }
        evidence = {
            "status": "isolated_humble_nav2_mppi_readiness_no_goal",
            "controller_plugin": "nav2_mppi_controller::MPPIController",
            "lifecycle": lifecycle,
            "follow_path_action_available": action_ready,
            "endpoints": endpoints,
            "controller_parameters": parameters,
            "local_costmap_parameters": costmap,
        }
        if (
            any(state != "active" for state in lifecycle.values())
            or not action_ready
            or endpoints["/cmd_vel"]["publishers"] != 1
            or endpoints["/cmd_vel"]["subscribers"] != 1
            or endpoints["/odom"]["publishers"] != 1
            or endpoints["/tf_static"]["publishers"] != 1
            or parameters["FollowPath.vx_max"] is None
            or not math.isclose(
                parameters["FollowPath.vx_max"], args.expected_vx_max, abs_tol=1e-9,
            )
            or not math.isclose(
                parameters["FollowPath.wz_max"], args.expected_wz_max, abs_tol=1e-9,
            )
            or parameters["FollowPath.time_steps"] != 30
            or not math.isclose(parameters["FollowPath.model_dt"], 0.10, abs_tol=1e-9)
            or parameters["FollowPath.motion_model"] not in ("DiffDrive", "diff_drive")
            or not math.isclose(parameters["controller_frequency"], 10.0, abs_tol=1e-9)
            or not math.isclose(costmap["robot_radius"], 0.61, abs_tol=1e-9)
            or costmap["track_unknown_space"] is not True
        ):
            raise AssertionError(f"MPPI readiness gate failed: {evidence}")
        evidence["status"] = "isolated_humble_nav2_mppi_readiness_pass_no_goal"
        rendered = json.dumps(evidence, indent=2, sort_keys=True) + "\n"
        if args.output is not None:
            if args.output.exists():
                raise FileExistsError(f"preserving existing readiness evidence: {args.output}")
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered, encoding="utf-8")
        print(rendered, end="")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
