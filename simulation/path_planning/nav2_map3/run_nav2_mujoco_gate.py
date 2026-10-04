#!/usr/bin/env python3
"""Drive the D1 Max MuJoCo proxy from isolated Jetson Nav2 /cmd_vel."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import numpy as np
import mujoco

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
import d1_edu_sb3_env as env_module  # noqa: E402
from d1_edu_sb3_env import D1DecisionEnv  # noqa: E402
from verify_nav2_map3_costmap import verify_costmap  # noqa: E402
from verify_nav2_map3_path import verify_path  # noqa: E402


def request_json(url: str, payload: dict | None = None, timeout_s: float = 1.0) -> dict:
    data = None if payload is None else json.dumps(payload, allow_nan=False).encode("utf-8")
    request = Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        with urlopen(request, timeout=timeout_s) as response:
            result = json.loads(response.read().decode("utf-8"))
            if not isinstance(result, dict):
                raise ValueError("bridge response must be a JSON object")
            return result
    except HTTPError as error:
        body = error.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"bridge HTTP {error.code}: {body}") from error
    except URLError as error:
        raise RuntimeError(f"bridge request failed: {error.reason}") from error


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-map-yaml", type=Path, required=True)
    parser.add_argument("--nav2-map-yaml", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--costmap-json", type=Path, required=True)
    parser.add_argument("--costmap-bin", type=Path, required=True)
    parser.add_argument("--nav2-path", type=Path, required=True)
    parser.add_argument("--expected-start-x", type=float, required=True)
    parser.add_argument("--expected-start-y", type=float, required=True)
    parser.add_argument("--start-yaw", type=float)
    parser.add_argument("--reset-seed", type=int, default=10042)
    parser.add_argument("--curriculum-index", type=int, default=0)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--bridge-url", default="http://127.0.0.1:18987")
    parser.add_argument("--sequence-start", type=int, default=6000)
    parser.add_argument("--active-goal-timeout", type=float, default=30.0)
    parser.add_argument("--wall-timeout", type=float, default=90.0)
    parser.add_argument("--controller-profile", choices=("rpp", "mppi"), default="rpp")
    parser.add_argument("--runtime-label")
    parser.add_argument("--controller-id")
    parser.add_argument("--controller-frequency-hz", type=float, default=10.0)
    parser.add_argument("--desired-linear-vel-mps", type=float, default=0.22)
    parser.add_argument("--max-forward-velocity-mps", type=float, default=0.22)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"preserving existing gate evidence: {args.output}")
    if (args.sequence_start < 0 or min(args.active_goal_timeout, args.wall_timeout) <= 0.0
            or not math.isfinite(args.controller_frequency_hz)
            or args.controller_frequency_hz <= 0.0
            or not math.isfinite(args.desired_linear_vel_mps)
            or args.desired_linear_vel_mps < 0.0
            or not math.isfinite(args.max_forward_velocity_mps)
            or args.max_forward_velocity_mps <= 0.0):
        raise ValueError("sequence and timeouts are invalid")

    if args.controller_profile == "rpp":
        runtime_label = args.runtime_label or (
            "Jetson Ubuntu 20.04 ROS 2 Foxy Nav2 controller_server + "
            "Mac MuJoCo D1 Max kinematic proxy"
        )
        controller_id = args.controller_id or (
            "nav2_regulated_pure_pursuit_controller/FollowPath"
        )
    else:
        runtime_label = args.runtime_label or (
            "Jetson ARM64 isolated ROS 2 Humble Nav2 container + "
            "Mac MuJoCo D1 Max kinematic proxy"
        )
        controller_id = args.controller_id or (
            "nav2_mppi_controller::MPPIController/FollowPath"
        )

    geometry = verify_path(
        args.nav2_map_yaml, args.manifest, args.nav2_path,
        expected_start=(args.expected_start_x, args.expected_start_y),
    )
    costmap = verify_costmap(
        args.nav2_map_yaml, args.manifest, args.costmap_json,
        args.costmap_bin, args.nav2_path,
    )
    route_evidence = json.loads(args.nav2_path.read_text(encoding="utf-8"))
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    if route_evidence.get("scene_id") != args.scene_id or manifest.get("scene_id") != args.scene_id:
        raise ValueError("Nav2 path, manifest, and bridge scene identities must match")
    path = np.asarray(route_evidence["points"], dtype=float)
    if len(path) < 2 or not np.isfinite(path).all():
        raise ValueError("Nav2 path needs at least two finite points")

    env = D1DecisionEnv(
        profile="baseline", robot_profile="d1_max_proxy", kinematic_proxy=True,
        scenario_mode="people_balanced_multi_target", seed=0, duration=70.0,
        route_length=10.0, mppi_samples=32, mppi_horizon=30,
        candidate_map_yaml=args.source_map_yaml, map_scene_scale=3.0,
        map_route_center_buffer_m=0.25,
    )
    sequence = args.sequence_start
    trace: list[dict[str, object]] = []
    status = "not_started"
    nav2_result_seen = False
    min_fixed_edge = math.inf
    command_histogram: dict[str, int] = {}
    contact_details: list[dict[str, object]] = []
    last_bridge_state: dict[str, object] | None = None
    last_report_second = -1
    start_wall = time.monotonic()

    def publish_pose() -> None:
        nonlocal sequence
        velocity = np.asarray(env.sdk.velocity, dtype=float)
        x, y = (float(value) for value in env.data.qpos[:2])
        request_json(args.bridge_url + "/pose", {
            "scene_id": args.scene_id,
            "sequence": sequence,
            "x": x, "y": y, "yaw": float(env.yaw),
            "vx": float(velocity[0]), "vy": float(velocity[1]),
            "wz": float(velocity[2]),
        })
        sequence += 1

    def disarm() -> None:
        try:
            request_json(args.bridge_url + "/disarm", {"scene_id": args.scene_id})
        except Exception as error:
            print(json.dumps({"disarm_error": str(error)}, sort_keys=True), flush=True)

    try:
        _, reset_info = env.reset(
            seed=args.reset_seed,
            options={"curriculum_index": args.curriculum_index},
        )
        actual_scene_id = f"reset{args.reset_seed}_scene{reset_info['map_scene_seed']}"
        if actual_scene_id != args.scene_id:
            raise ValueError(f"MuJoCo reset produced {actual_scene_id}, expected {args.scene_id}")
        fixed = [item for item in env.trajectories if item.kind != "person"]
        people = [item for item in env.trajectories if item.kind == "person"]
        if len(fixed) != 1 or len(people) != 1:
            raise ValueError("expected one fixed object and one pedestrian")
        box = manifest["static_box"]
        if any(abs(float(getattr(fixed[0], key)) - float(box[key])) > 1e-8
               for key in ("x", "y", "half_length", "half_width")):
            raise ValueError("MuJoCo fixed object differs from the Nav2 map manifest")
        env.trajectories = [
            replace(item, appear_time=math.inf) if item.kind == "person" else item
            for item in env.trajectories
        ]
        for body, trajectory in zip(env.obstacle_bodies, env.trajectories):
            body.trajectory = trajectory
        env.tracker.tracks.clear()
        env.tracker.next_track_id = 0
        env.tracker.selected_track = None
        env._update_bodies(0.0)

        reset_start_xy = [float(value) for value in env.data.qpos[:2]]
        if not env.map_planner.is_traversable_world(float(path[0, 0]), float(path[0, 1])):
            raise ValueError("Nav2 route start is outside inflated known-free space")
        if not env.map_planner.is_traversable_world(float(path[-1, 0]), float(path[-1, 1])):
            raise ValueError("Nav2 route goal is outside inflated known-free space")
        env.data.qpos[:2] = path[0]
        if args.start_yaw is not None:
            if not math.isfinite(args.start_yaw):
                raise ValueError("start-yaw must be finite")
            env.yaw = float(args.start_yaw)
            env.data.qpos[3:7] = env_module.yaw_to_quat(env.yaw)
        mujoco.mj_forward(env.model, env.data)
        initial_yaw = float(env.yaw)
        env.goal_world = path[-1].copy()
        env.config = replace(env.config, goal_tolerance=0.30)
        env.global_path.set_path(path)
        env.reached_goal = False
        env._update_bodies(0.0)
        x0, y0 = (float(value) for value in env.data.qpos[:2])
        if not env.map_planner.is_traversable_world(x0, y0):
            raise RuntimeError("aligned MuJoCo start is outside inflated known-free space")
        initial_clearances = [
            env_module.collision_clearance(
                env.data.qpos[:2], state[0], obstacle,
                robot_radius=env.global_robot_radius,
            )
            for state, obstacle in zip(env._current_states(), env.obstacle_specs)
            if state[2]
        ]
        if initial_clearances and min(initial_clearances) < -env.config.collision_tolerance:
            raise RuntimeError("aligned MuJoCo start overlaps an active obstacle")
        env.min_clearance = min(initial_clearances) if initial_clearances else math.inf
        env.geometric_collision = bool(
            env.min_clearance < -env.config.collision_tolerance
        )
        env.physical_contact, _, _ = env_module.mujoco_obstacle_contacts(
            env.model, env.data, env.obstacle_geom_ids,
        )
        env.collision = bool(env.physical_contact)
        if env.collision or env.geometric_collision:
            raise RuntimeError("MuJoCo start has a collision/contact")

        def fixed_edge_distance() -> float:
            x, y = (float(value) for value in env.data.qpos[:2])
            return math.hypot(
                max(abs(x - float(box["x"])) - float(box["half_length"]), 0.0),
                max(abs(y - float(box["y"])) - float(box["half_width"]), 0.0),
            )

        min_fixed_edge = fixed_edge_distance()
        if min_fixed_edge < 0.61:
            raise RuntimeError("aligned MuJoCo start is inside fixed-object clearance margin")

        # Start a fresh sequence well above the bounded stationary bring-up feed.
        publish_pose()
        active_deadline = time.monotonic() + args.active_goal_timeout
        while time.monotonic() < active_deadline:
            state = request_json(args.bridge_url + "/health")
            if state.get("scene_id") != args.scene_id:
                raise RuntimeError("bridge scene identity changed")
            if state.get("goal_active"):
                break
            publish_pose()
            time.sleep(0.05)
        else:
            raise TimeoutError("no active Nav2 FollowPath goal appeared")

        armed = request_json(args.bridge_url + "/arm", {"scene_id": args.scene_id})
        if not armed.get("armed"):
            raise RuntimeError(f"bridge refused to arm: {armed}")
        status = "running"
        print(json.dumps({
            "status": status, "scene_id": args.scene_id,
            "start_xy": [float(v) for v in env.data.qpos[:2]],
            "goal_xy": [float(v) for v in env.goal_world],
            "sim_dt": env.sim_dt,
            "policy_period": env.config.policy_period,
            "duration_s": env.config.duration,
            "fixed_box_edge_clearance_gate_m": 0.61,
        }, sort_keys=True), flush=True)

        loop_start = time.monotonic()
        while env.elapsed < env.config.duration - 1e-9:
            if time.monotonic() - loop_start > args.wall_timeout:
                status = "wall_timeout"
                break
            publish_pose()
            action_state = request_json(args.bridge_url + "/health")
            last_bridge_state = dict(action_state)
            if not action_state.get("goal_active"):
                action_status = action_state.get("last_goal_status_code")
                if action_status is not None:
                    nav2_result_seen = True
                    status = "nav2_succeeded" if action_status == 4 else "nav2_action_failed"
                    break
                time.sleep(0.05)
                continue
            state = request_json(args.bridge_url + "/command")
            last_bridge_state = dict(state)
            if not state.get("armed") or not state.get("goal_active"):
                action_status = state.get("last_goal_status_code")
                if not state.get("goal_active") and action_status is not None:
                    nav2_result_seen = True
                    status = "nav2_succeeded" if action_status == 4 else "nav2_action_failed"
                else:
                    status = "bridge_disarmed_while_goal_active"
                break
            command = np.asarray(state["velocity_body"], dtype=float)
            if command.shape != (3,) or not np.isfinite(command).all():
                status = "invalid_bridge_command"
                break
            command_key = ",".join(f"{value:.2f}" for value in command)
            command_histogram[command_key] = command_histogram.get(command_key, 0) + 1
            # Nav2's action result is the episode stop condition. Keep stepping
            # when MuJoCo first crosses the matching XY tolerance so the
            # controller can observe that pose and report success itself.
            env.reached_goal = False
            env._advance(command)
            edge = fixed_edge_distance()
            min_fixed_edge = min(min_fixed_edge, edge)
            elapsed_second = int(env.elapsed)
            if elapsed_second > last_report_second and elapsed_second % 2 == 0:
                distance = float(np.linalg.norm(env.data.qpos[:2] - env.goal_world))
                trace.append({
                    "elapsed_s": float(env.elapsed),
                    "robot_xy": [float(v) for v in env.data.qpos[:2]],
                    "robot_yaw_rad": float(env.yaw),
                    "nav2_cmd_body": [float(v) for v in command],
                    "goal_distance_m": distance,
                    "fixed_box_edge_distance_m": edge,
                    "local_goal_tolerance_reached": bool(env.reached_goal),
                    "map_traversable": bool(env.map_planner.is_traversable_world(
                        float(env.data.qpos[0]), float(env.data.qpos[1]),
                    )),
                })
                last_report_second = elapsed_second
                print(json.dumps({
                    "elapsed_s": round(float(env.elapsed), 2),
                    "goal_distance_m": round(distance, 3),
                    "fixed_box_edge_m": round(edge, 3),
                    "collision": bool(env.collision),
                    "map_violation": bool(env.map_violation),
                }, sort_keys=True), flush=True)
            if env.collision or env.geometric_collision or env.map_violation:
                status = "collision" if env.collision or env.geometric_collision else "map_violation"
                disarm()
                break
            sim_elapsed = float(env.elapsed)
            target_wall = loop_start + sim_elapsed
            delay = target_wall - time.monotonic()
            if delay > 0.0:
                time.sleep(min(delay, 0.05))
        if env.elapsed >= env.config.duration - 1e-9 and not env.reached_goal:
            status = "timeout"
        disarm()
        terminal_distance = float(np.linalg.norm(env.data.qpos[:2] - env.goal_world))
        final_state = request_json(args.bridge_url + "/health")
        nav2_result_status = final_state.get("last_goal_status_code")
        if nav2_result_status in (4, 5, 6):
            nav2_result_seen = True
            if status in ("running", "bridge_disarmed_while_goal_active"):
                status = "nav2_succeeded" if nav2_result_status == 4 else "nav2_action_failed"
        if status == "nav2_succeeded" and nav2_result_status == 4:
            status = "goal_reached_nav2_succeeded"
        contacts = []
        for contact in env.data.contact[:env.data.ncon]:
            first, second = int(contact.geom1), int(contact.geom2)
            if (first in env.obstacle_geom_ids or second in env.obstacle_geom_ids):
                if env.model.geom(first).name != "floor" and env.model.geom(second).name != "floor":
                    contacts.append({
                        "geom_a": env.model.geom(first).name,
                        "geom_b": env.model.geom(second).name,
                        "penetration_m": float(contact.dist),
                    })
        contact_details = contacts
        passed = bool(
            status == "goal_reached_nav2_succeeded"
            and not env.collision and not env.geometric_collision
            and not env.map_violation and min_fixed_edge >= 0.61
            and terminal_distance <= 0.30
        )
        evidence = {
            "status": f"nav2_{args.controller_profile}_mujoco_fixed_gate_{'pass' if passed else 'fail'}",
            "episode_stop_reason": status,
            "bridge_state_at_stop": last_bridge_state,
            "scene_id": args.scene_id,
            "runtime": runtime_label,
            "controller": controller_id,
            "controller_frequency_hz": args.controller_frequency_hz,
            "robot_proxy_radius_m": 0.49,
            "required_obstacle_edge_clearance_m": 0.61,
            "static_unknown_blocked": True,
            "pedestrian_disabled": True,
            "fixed_object_active": True,
            "reset_seed": args.reset_seed,
            "curriculum_index": args.curriculum_index,
            "episode_seed": int(reset_info["episode_seed"]),
            "map_scene_seed": int(reset_info["map_scene_seed"]),
            "simulation_reset_start_xy": reset_start_xy,
            "nav2_path_start_xy": [float(v) for v in path[0]],
            "initial_yaw_rad": initial_yaw,
            "source_map_yaml_sha256": hashlib.sha256(args.source_map_yaml.read_bytes()).hexdigest(),
            "nav2_map_yaml_sha256": hashlib.sha256(args.nav2_map_yaml.read_bytes()).hexdigest(),
            "nav2_path_sha256": hashlib.sha256(args.nav2_path.read_bytes()).hexdigest(),
            "costmap_sha256": json.loads(args.costmap_json.read_text(encoding="utf-8"))["costmap_sha256"],
            "preflight_path": geometry,
            "preflight_costmap": costmap,
            "sim_elapsed_s": float(env.elapsed),
            "wall_elapsed_s": time.monotonic() - start_wall,
            "nav2_terminal_transition_observed": nav2_result_seen,
            "nav2_result_status_code": nav2_result_status,
            "goal_distance_m": terminal_distance,
            "success": bool(passed),
            "physical_contact": bool(env.physical_contact),
            "geometric_collision": bool(env.geometric_collision),
            "map_violation": bool(env.map_violation),
            "timeout": bool(status in ("timeout", "wall_timeout")),
            "min_fixed_box_edge_distance_m": min_fixed_edge,
            "command_sample_count": sum(command_histogram.values()),
            "command_histogram_rounded": command_histogram,
            "mode_trace_0p5hz": trace,
            "terminal_contacts": contact_details,
            "note": "MuJoCo uses a D1 Max kinematic proxy and the existing delayed/saturated simulated velocity interface; this is not gait or hardware evidence.",
        }
        if args.controller_profile == "rpp":
            evidence["desired_linear_vel_mps"] = args.desired_linear_vel_mps
        else:
            evidence["max_forward_velocity_mps"] = args.max_forward_velocity_mps
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({key: evidence[key] for key in (
            "status", "episode_stop_reason", "bridge_state_at_stop",
            "sim_elapsed_s", "wall_elapsed_s", "success", "physical_contact",
            "geometric_collision", "map_violation", "timeout", "goal_distance_m",
            "min_fixed_box_edge_distance_m", "nav2_terminal_transition_observed",
        )}, sort_keys=True), flush=True)
    except Exception:
        disarm()
        raise
    finally:
        disarm()
        env.close()


if __name__ == "__main__":
    main()
