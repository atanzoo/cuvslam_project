#!/usr/bin/env python3
"""Generate a leakage-safe IMM dataset through the real simulated-LiDAR path.

Runtime input is produced by MuJoCo rangefinders, the existing LiDAR noise /
dropout model, and ``MultiTargetTracker``.  Analytic obstacle state is used
only for future labels and evaluation.  Splits are by episode seed, never by
adjacent windows from the same episode.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from d1_edu_scenario import generate_people_random_multi_target_scenario
from run_d1_edu_decision_layer_viewer import (
    DynamicObstacleBody,
    MultiTargetTracker,
    apply_lidar_model,
)
from run_d1_edu_mppi_obstacle_viewer import SimulatedLidar
from run_d1_edu_obstacle_avoidance_viewer import Obstacle, build_scene


MOTION_KINDS = ("static", "crossing", "same_direction", "accelerating", "turning", "irregular")


def _estimate_kinematics(observed: np.ndarray, dt_or_times: float | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Fit a local quadratic instead of taking noisy second differences."""
    filled = np.asarray(observed, dtype=float).copy()
    for axis in range(2):
        valid = np.isfinite(filled[:, axis])
        if not np.any(valid):
            filled[:, axis] = 0.0
            continue
        valid_values = np.flatnonzero(valid)
        for index in range(len(filled)):
            if not valid[index]:
                nearest = valid_values[np.argmin(np.abs(valid_values - index))]
                filled[index, axis] = filled[nearest, axis]
    if np.isscalar(dt_or_times):
        times = np.arange(-len(filled) + 1, 1, dtype=float) * float(dt_or_times)
    else:
        times = np.asarray(dt_or_times, dtype=float).reshape(-1)
        times = times - times[-1]
    velocity = np.zeros(2)
    acceleration = np.zeros(2)
    for axis in range(2):
        degree = min(2, len(times) - 1)
        coefficients = np.polyfit(times, filled[:, axis], degree)
        if degree == 2:
            acceleration[axis] = 2.0 * coefficients[0]
            velocity[axis] = 2.0 * coefficients[0] * times[-1] + coefficients[1]
        else:
            velocity[axis] = coefficients[0]
    return velocity, acceleration


def _sample(seed: int, kind: str, *, history: int, dt: float, horizons: np.ndarray,
            noise: float, dropout: float, duration: float) -> dict[str, np.ndarray]:
    """Run one target through MuJoCo LiDAR and the production tracker."""
    scenario = generate_people_random_multi_target_scenario(
        seed, route_length=4.0, max_obstacle_speed=1.5,
        motion_schedule=(kind, kind, kind),
    )
    trajectory = scenario.obstacles[0]
    obstacle_spec = Obstacle(
        x=trajectory.x,
        y=trajectory.y,
        half_length=trajectory.half_length,
        half_width=trajectory.half_width,
        height=trajectory.height,
        kind=trajectory.kind,
    )
    model, _ = build_scene(
        with_lidar=True,
        dynamic_obstacles=[obstacle_spec],
        lidar_ray_count=61,
        lidar_fov=np.pi,
        lidar_max_range=4.0,
        world_length=4.0,
    )
    data = mujoco.MjData(model)
    data.qpos[2] = 0.335
    body = DynamicObstacleBody(model, trajectory, index=0)
    body.update(data, 0.0)
    mujoco.mj_forward(model, data)
    lidar = SimulatedLidar(model, ray_count=61, max_range=4.0, fov=np.pi)
    tracker = MultiTargetTracker(
        obstacle_half_length=trajectory.half_length,
        obstacle_half_width=trajectory.half_width,
        max_track_age=0.45,
        velocity_alpha=0.30,
    )
    rng = np.random.default_rng(seed * 1009 + MOTION_KINDS.index(kind))
    rows: list[dict[str, np.ndarray | float | int | str]] = []
    simulation_times = np.arange(0.0, duration + 1e-9, dt)
    for elapsed in simulation_times:
        body.update(data, float(elapsed))
        mujoco.mj_forward(model, data)
        raw_scan = lidar.scan(data)
        scan = apply_lidar_model(raw_scan, rng, noise, dropout, lidar.max_range)
        points = lidar.point_cloud(scan)
        ray_indices = lidar.hit_ray_indices(scan)
        tracks = tracker.update(
            points,
            np.zeros(2, dtype=float),
            0.0,
            float(elapsed),
            ray_indices,
        )
        current_position, current_velocity, active = trajectory.state(float(elapsed))
        if not active:
            continue
        for track in tracks:
            if np.linalg.norm(track.position_world - current_position) > 0.80:
                continue
            if not track.history or float(track.history[-1][0]) < elapsed - dt * 1.5:
                continue
            if len(track.history) < history + 1:
                continue
            history_rows = track.history[-(history + 1):]
            observed_times = np.array([row[0] for row in history_rows], dtype=float)
            observed = np.array([row[1] for row in history_rows], dtype=float)
            velocity, acceleration = _estimate_kinematics(observed, observed_times)
            future_times = elapsed + horizons
            future = np.array([trajectory.state(float(t))[0] for t in future_times])
            future_v = np.array([trajectory.state(float(t))[1] for t in future_times])
            previous_velocity = trajectory.state(max(0.0, elapsed - dt))[1]
            next_velocity = trajectory.state(elapsed + dt)[1]
            true_a = (next_velocity - previous_velocity) / max(2.0 * dt, 1e-6)
            speed2 = max(float(current_velocity @ current_velocity), 1e-6)
            true_turn = float((current_velocity[0] * true_a[1]
                               - current_velocity[1] * true_a[0]) / speed2)
            future_clearance = np.abs(future[:, 1]) - trajectory.half_width - 0.25
            rows.append({
                "features": np.array([
                    *observed.reshape(-1), *velocity, *acceleration,
                    float(track.confidence), float(track.miss_count), noise, dropout,
                ]),
                "position": track.position_world.copy(), "velocity": velocity,
                "acceleration": acceleration, "future": future,
                "future_velocity": future_v, "turn_rate": true_turn,
                "risk": float(np.min(future_clearance) <= 0.0),
                "clearance": float(np.min(future_clearance)),
                "future_clearance": future_clearance, "confidence": float(track.confidence),
                "miss_count": float(track.miss_count), "noise": noise, "dropout": dropout,
                "kind": kind,
            })
    if not rows:
        raise ValueError(f"simulated LiDAR produced no usable {kind} tracks for seed {seed}")
    return {key: np.array([row[key] for row in rows]) for key in rows[0] if key != "kind"} | {
        "kind": np.array([kind] * len(rows)), "seed": np.array([seed] * len(rows), dtype=np.int64)
    }


def generate_dataset(episodes: int = 180, seed: int = 7, *, history: int = 10,
                     dt: float = 0.1, noise: float = 0.01, dropout: float = 0.04,
                     duration: float = 8.0) -> tuple[dict[str, np.ndarray], dict]:
    if episodes < 12 or history < 3 or dt <= 0 or not 0 <= dropout < 1:
        raise ValueError("episodes>=12, history>=3, dt>0, and dropout in [0,1) are required")
    chunks = [_sample(seed + i, MOTION_KINDS[i % len(MOTION_KINDS)], history=history,
                      dt=dt, horizons=np.array([0.5, 1.0, 2.0, 3.0]), noise=noise,
                      dropout=dropout, duration=duration) for i in range(episodes)]
    all_seeds = np.arange(seed, seed + episodes)
    rng = np.random.default_rng(seed)
    rng.shuffle(all_seeds)
    n_train, n_val = int(0.70 * episodes), int(0.15 * episodes)
    split = {"train": all_seeds[:n_train], "val": all_seeds[n_train:n_train+n_val],
             "test": all_seeds[n_train+n_val:]}
    keys = [k for k in chunks[0] if k not in {"kind", "seed"}]
    data = {key: np.concatenate([c[key] for c in chunks]) for key in keys}
    episode_seed = np.concatenate([c["seed"] for c in chunks])
    data["kind"] = np.concatenate([c["kind"] for c in chunks])
    data["episode_seed"] = episode_seed
    data["split"] = np.array(["train" if s in split["train"] else "val" if s in split["val"] else "test"
                               for s in episode_seed])
    manifest = {"episodes": episodes, "seed": seed, "history": history, "dt": dt,
                "noise": noise, "dropout": dropout, "duration": duration,
                "horizons": [0.5, 1.0, 2.0, 3.0], "motion_kinds": list(MOTION_KINDS),
                "source": "mujoco_simulated_lidar_multi_target_tracker",
                "split_episode_seeds": {k: v.tolist() for k, v in split.items()},
                "samples": int(len(episode_seed))}
    return data, manifest


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=180)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--history", type=int, default=10)
    ap.add_argument("--noise", type=float, default=0.01)
    ap.add_argument("--dropout", type=float, default=0.04)
    ap.add_argument("--output", type=Path, default=Path("output/d1_imm_dataset.npz"))
    ap.add_argument("--manifest", type=Path, default=Path("output/d1_imm_dataset_manifest.json"))
    args = ap.parse_args()
    data, manifest = generate_dataset(args.episodes, args.seed, history=args.history,
                                       noise=args.noise, dropout=args.dropout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, **data)
    args.manifest.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {args.output} samples={manifest['samples']} split={manifest['split_episode_seeds']}")


if __name__ == "__main__":
    main()
