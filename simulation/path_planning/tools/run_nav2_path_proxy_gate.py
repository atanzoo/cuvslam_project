#!/usr/bin/env python3
"""Run one local MuJoCo proxy with a previously verified real Nav2 path.

This is a partial closed-loop gate: the Nav2 planner path is used by the
existing local decision/MPPI/SDK chain, but no Nav2 controller or ROS bridge
is present during the rollout. It must not authorize PPO training by itself.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from d1_edu_sb3_env import D1DecisionEnv
from d1_edu_nav2_global_path import path_heading_command
from verify_nav2_map3_path import verify_path
from verify_nav2_map3_costmap import verify_costmap


def run(args: argparse.Namespace) -> dict[str, object]:
    if args.output.exists():
        raise FileExistsError(f"gate evidence already exists: {args.output}")
    geometry = verify_path(
        args.nav2_map_yaml, args.manifest, args.nav2_path,
        expected_start=(args.expected_start_x, args.expected_start_y),
    )
    costmap = verify_costmap(
        args.nav2_map_yaml, args.manifest, args.costmap_json,
        args.costmap_bin, args.nav2_path,
    )
    route = json.loads(args.nav2_path.read_text(encoding="utf-8"))
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    path = np.asarray(route["points"], dtype=float)
    env = D1DecisionEnv(
        profile="baseline", robot_profile="d1_max_proxy", kinematic_proxy=True,
        scenario_mode="people_balanced_multi_target", seed=0, duration=70.0,
        route_length=10.0, mppi_samples=32, mppi_horizon=30,
        candidate_map_yaml=args.source_map_yaml, map_scene_scale=3.0,
        map_route_center_buffer_m=0.25,
    )
    original_command_for_action = env._command_for_action
    try:
        _, reset_info = env.reset(seed=args.reset_seed, options={"curriculum_index": args.curriculum_index})
        expected_id = f"reset{args.reset_seed}_scene{reset_info['map_scene_seed']}"
        if expected_id != manifest["scene_id"]:
            raise ValueError(f"reset scene {expected_id} differs from Nav2 map {manifest['scene_id']}")
        if np.linalg.norm(env.active_map_path[0] - path[0]) > 0.10:
            raise ValueError("Nav2 route start differs from MuJoCo reset start")
        if np.linalg.norm(env.goal_world - path[-1]) > 0.10:
            raise ValueError("Nav2 route goal differs from MuJoCo reset goal")
        fixed = [item for item in env.trajectories if item.kind != "person"]
        people = [item for item in env.trajectories if item.kind == "person"]
        if len(fixed) != 1 or len(people) != 1:
            raise ValueError("expected exactly one fixed object and one pedestrian")
        box = manifest["static_box"]
        if any(abs(float(getattr(fixed[0], key)) - float(box[key])) > 1e-8
               for key in ("x", "y", "half_length", "half_width")):
            raise ValueError("MuJoCo fixed-object geometry differs from Nav2 derived map")
        # Retain the physical fixed obstacle; disable only the pedestrian for
        # this first safety gate. No collision predicate or radius is relaxed.
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
        env.global_path.set_path(path)
        env._observe()
        if args.experimental_avoid_heading:
            def experimental_command_for_action(action: int) -> np.ndarray:
                command = original_command_for_action(action)
                if env.active_mode == "AVOID":
                    command[2] = path_heading_command(
                        env.latest_path_reference,
                        gain=1.8,
                        max_rate=env.config.max_yaw_rate,
                    )
                return command
            env._command_for_action = experimental_command_for_action
        modes: Counter[str] = Counter()
        trace: list[dict[str, object]] = []
        last_bin = -1
        steps = 0
        min_fixed_edge = math.inf
        terminal = {}
        while True:
            _, _, terminated, truncated, terminal = env.step(0)
            steps += 1
            modes[str(terminal["mode"])] += 1
            x, y = (float(value) for value in env.data.qpos[:2])
            edge = math.hypot(
                max(abs(x - float(box["x"])) - float(box["half_length"]), 0.0),
                max(abs(y - float(box["y"])) - float(box["half_width"]), 0.0),
            )
            min_fixed_edge = min(min_fixed_edge, edge)
            second_bin = int(terminal["elapsed_time"])
            if second_bin != last_bin or terminated or truncated:
                trace.append({
                    "elapsed_s": float(terminal["elapsed_time"]),
                    "robot_xy": [x, y],
                    "mode": str(terminal["mode"]),
                    "baseline_mode": str(env.latest_baseline_mode),
                    "risk": bool(terminal["risk"]),
                    "ttc_s": float(env.latest_ttc) if math.isfinite(float(env.latest_ttc)) else None,
                    "avoid_side": float(env.planner.avoid_side) if env.planner.avoid_side is not None else None,
                    "sdk_velocity_body": [float(value) for value in env.sdk.velocity],
                    "goal_distance_m": float(terminal["distance_to_goal"]),
                    "fixed_edge_distance_m": edge,
                    "path_progress": float(terminal["path_progress"]),
                    "path_heading_error_rad": float(terminal["path_heading_error"]),
                    "path_lateral_error_m": float(terminal["path_lateral_error"]),
                })
                last_bin = second_bin
            if terminated or truncated:
                break
        contacts = []
        for contact in env.data.contact[:env.data.ncon]:
            first, second = int(contact.geom1), int(contact.geom2)
            if (
                (first in env.obstacle_geom_ids or second in env.obstacle_geom_ids)
                and env.model.geom(first).name != "floor"
                and env.model.geom(second).name != "floor"
            ):
                contacts.append({
                    "geom_a": env.model.geom(first).name,
                    "geom_b": env.model.geom(second).name,
                    "penetration_m": float(contact.dist),
                    "position_xyz": [float(value) for value in contact.pos],
                })
        passed = bool(
            terminal.get("success")
            and not terminal.get("collision")
            and not terminal.get("map_violation")
            and not terminal.get("timeout")
            and min_fixed_edge >= 0.61
        )
        evidence = {
            "status": "partial_nav2_path_fed_proxy_gate_pass" if passed else "partial_nav2_path_fed_proxy_gate_fail",
            "boundary": "real Nav2 global path injected; local MuJoCo/MPPI control; no ROS/Nav2 controller during rollout",
            "controller_variant": (
                "experimental_avoid_heading_in_memory_only"
                if args.experimental_avoid_heading else "unchanged_baseline"
            ),
            "scene_id": manifest["scene_id"],
            "reset_seed": args.reset_seed,
            "curriculum_index": args.curriculum_index,
            "episode_seed": reset_info["episode_seed"],
            "map_scene_seed": reset_info["map_scene_seed"],
            "pedestrian_disabled": True,
            "fixed_object_active": True,
            "source_map_yaml_sha256": hashlib.sha256(args.source_map_yaml.read_bytes()).hexdigest(),
            "nav2_path_sha256": hashlib.sha256(args.nav2_path.read_bytes()).hexdigest(),
            "costmap_sha256": json.loads(args.costmap_json.read_text())["costmap_sha256"],
            "preflight_path": geometry,
            "preflight_costmap": costmap,
            "steps": steps,
            "mode_counts": dict(modes),
            "success": bool(terminal.get("success", False)),
            "collision": bool(terminal.get("collision", False)),
            "physical_contact": bool(terminal.get("physical_contact", False)),
            "map_violation": bool(terminal.get("map_violation", False)),
            "timeout": bool(terminal.get("timeout", False)),
            "arrival_time_s": float(terminal["elapsed_time"]) if terminal.get("reached_goal") else None,
            "terminal_goal_distance_m": float(terminal["distance_to_goal"]),
            "min_fixed_edge_distance_m": min_fixed_edge,
            "mode_trace_1hz": trace,
            "terminal_obstacle_contacts": contacts,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return {key: evidence[key] for key in (
            "status", "scene_id", "steps", "success", "collision", "map_violation",
            "timeout", "arrival_time_s", "terminal_goal_distance_m", "min_fixed_edge_distance_m",
        )}
    finally:
        env._command_for_action = original_command_for_action
        env.close()


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
    parser.add_argument("--reset-seed", type=int, required=True)
    parser.add_argument("--curriculum-index", type=int, required=True)
    parser.add_argument("--experimental-avoid-heading", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
