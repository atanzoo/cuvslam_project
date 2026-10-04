#!/usr/bin/env python3
"""Scenario generation for long-route D1 Edu obstacle experiments.

The generator is intentionally independent from MuJoCo.  It produces
replayable, constant-parameter obstacle trajectories that the MuJoCo episode
runner can attach to free-joint boxes.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any

import numpy as np


SCENARIO_KINDS = (
    "static",
    "crossing",
    "same_direction",
    "accelerating",
    "turning",
    "sudden_appearance",
)


@dataclass(frozen=True)
class ScenarioObstacle:
    """One obstacle footprint and its analytic planar trajectory.

    ``kind == "person"`` is a pedestrian proxy: the planner still reasons
    about its planar footprint, while MuJoCo renders it as a tall cylinder.
    This is intentionally not a full human-body or leg-motion model.
    """

    obstacle_id: str
    kind: str
    x: float
    y: float
    half_length: float = 0.25
    half_width: float = 0.25
    vx: float = 0.0
    vy: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    turn_center_x: float = 0.0
    turn_center_y: float = 0.0
    turn_radius: float = 1.0
    turn_phase: float = 0.0
    turn_rate: float = 0.0
    irregular_lateral_amplitude: float = 0.0
    irregular_frequency: float = 0.0
    irregular_phase: float = 0.0
    irregular_secondary_amplitude: float = 0.0
    irregular_secondary_frequency: float = 0.0
    irregular_secondary_phase: float = 0.0
    appear_time: float = 0.0
    disappear_time: float = math.inf
    # Physical type and motion type are separate. Existing scenarios use
    # ``kind`` for both; randomized people scenes set kind to ``person`` or
    # ``object`` and put the sampled motion family here.
    motion_kind: str | None = None
    # MuJoCo geom half-height. Historical boxes use 0.25 m; a person proxy
    # uses about 0.85 m (1.7 m total height).
    height: float = 0.25
    max_speed: float = 7.0

    def active(self, elapsed: float) -> bool:
        return self.appear_time <= elapsed <= self.disappear_time

    def state(self, elapsed: float) -> tuple[np.ndarray, np.ndarray, bool]:
        """Return true position, true velocity and active state."""
        if not self.active(elapsed):
            return np.array([self.x, self.y], dtype=float), np.zeros(2), False
        tau = max(0.0, elapsed - self.appear_time)
        motion_kind = self.motion_kind or self.kind
        if motion_kind == "turning":
            theta = self.turn_phase + self.turn_rate * tau
            position = np.array([
                self.turn_center_x + self.turn_radius * np.cos(theta),
                self.turn_center_y + self.turn_radius * np.sin(theta),
            ])
            velocity = np.array([
                -self.turn_radius * self.turn_rate * np.sin(theta),
                self.turn_radius * self.turn_rate * np.cos(theta),
            ])
        elif motion_kind in {"irregular", "person"}:
            base_x = self.x + self.vx * tau + 0.5 * self.ax * tau * tau
            base_y = self.y + self.vy * tau + 0.5 * self.ay * tau * tau
            phase = self.irregular_phase + self.irregular_frequency * tau
            secondary_phase = (
                self.irregular_secondary_phase
                + self.irregular_secondary_frequency * tau
            )
            lateral_offset = (
                self.irregular_lateral_amplitude * np.sin(phase)
                + self.irregular_secondary_amplitude * np.sin(secondary_phase)
            )
            lateral_velocity = (
                self.irregular_lateral_amplitude
                * self.irregular_frequency
                * np.cos(phase)
                + self.irregular_secondary_amplitude
                * self.irregular_secondary_frequency
                * np.cos(secondary_phase)
            )
            position = np.array([base_x, base_y + lateral_offset])
            velocity = np.array([
                self.vx + self.ax * tau,
                self.vy + self.ay * tau + lateral_velocity,
            ])
        else:
            position = np.array([
                self.x + self.vx * tau + 0.5 * self.ax * tau * tau,
                self.y + self.vy * tau + 0.5 * self.ay * tau * tau,
            ])
            velocity = np.array([
                self.vx + self.ax * tau,
                self.vy + self.ay * tau,
            ])
        speed = float(np.linalg.norm(velocity))
        if speed > self.max_speed > 0.0:
            velocity *= self.max_speed / speed
        if motion_kind != "turning" and self.max_speed > 0.0:
            displacement = position - np.array([self.x, self.y], dtype=float)
            displacement_norm = float(np.linalg.norm(displacement))
            maximum_displacement = self.max_speed * tau
            if displacement_norm > maximum_displacement > 0.0:
                position = np.array([self.x, self.y], dtype=float)
                position += displacement * (maximum_displacement / displacement_norm)
        return position, velocity, True

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_mapping(cls, mapping: dict[str, Any]) -> "ScenarioObstacle":
        allowed = cls.__dataclass_fields__
        return cls(**{key: value for key, value in mapping.items() if key in allowed})


@dataclass(frozen=True)
class LongRouteScenario:
    route_length: float
    max_obstacle_speed: float
    obstacles: tuple[ScenarioObstacle, ...]
    lidar_model: str = "RPLIDAR_A2M8"
    lidar_scan_rate_hz: float = 10.0
    lidar_max_range: float = 12.0
    lidar_angular_resolution_deg: float = 0.45

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["obstacles"] = [obstacle.to_dict() for obstacle in self.obstacles]
        return payload


def _bounded_speed(rng: np.random.Generator, maximum: float) -> float:
    return float(rng.uniform(0.20, min(1.20, maximum)))


def generate_long_route_scenario(
    seed: int,
    route_length: float = 30.0,
    max_obstacle_speed: float = 7.0,
    obstacle_count: int = 6,
) -> LongRouteScenario:
    """Generate all requested obstacle families on a 30 m route.

    One instance of every requested behavior is included; additional objects
    are sampled as static/crossing/moving obstacles when ``obstacle_count``
    exceeds six.  Typical laboratory speeds stay below 1.2 m/s, while the
    explicit hard cap remains 7 m/s for stress cases.
    """
    if route_length <= 0.0:
        raise ValueError("route_length must be positive")
    if max_obstacle_speed <= 0.0 or max_obstacle_speed > 7.0:
        raise ValueError("max_obstacle_speed must be in (0, 7]")
    if obstacle_count < len(SCENARIO_KINDS):
        raise ValueError("obstacle_count must include all requested scenario kinds")

    rng = np.random.default_rng(seed)
    obstacles: list[ScenarioObstacle] = []
    for index, kind in enumerate(SCENARIO_KINDS):
        x = float(rng.uniform(4.0, max(5.0, route_length - 3.0)))
        y = float(rng.uniform(-2.0, 2.0))
        half_length = float(rng.uniform(0.18, 0.40))
        half_width = float(rng.uniform(0.18, 0.40))
        speed = _bounded_speed(rng, max_obstacle_speed)
        kwargs: dict[str, Any] = {
            "obstacle_id": f"obstacle_{index:02d}_{kind}",
            "kind": kind,
            "x": x,
            "y": y,
            "half_length": half_length,
            "half_width": half_width,
            "max_speed": max_obstacle_speed,
        }
        if kind == "crossing":
            crossing_y = -4.0 if rng.random() < 0.5 else 4.0
            kwargs.update({
                "x": float(rng.uniform(5.0, route_length - 5.0)),
                "y": crossing_y,
                "vx": float(rng.uniform(-0.10, 0.10)),
                "vy": (speed if crossing_y < 0.0 else -speed),
            })
        elif kind == "same_direction":
            kwargs.update({
                "x": float(rng.uniform(5.0, route_length - 5.0)),
                "y": float(rng.uniform(-1.0, 1.0)),
                "vx": float(rng.uniform(0.10, min(0.70, max_obstacle_speed))),
                "vy": float(rng.uniform(-0.10, 0.10)),
            })
        elif kind == "accelerating":
            kwargs.update({
                "x": float(rng.uniform(6.0, route_length - 6.0)),
                "y": float(rng.uniform(-1.4, 1.4)),
                "vx": float(rng.uniform(0.05, min(0.50, max_obstacle_speed))),
                "ax": float(rng.uniform(0.03, min(0.20, max_obstacle_speed / 4.0))),
                "ay": float(rng.uniform(-0.04, 0.04)),
            })
        elif kind == "turning":
            radius = float(rng.uniform(0.8, 2.0))
            turn_speed = float(rng.uniform(0.20, min(1.0, max_obstacle_speed)))
            kwargs.update({
                "turn_center_x": x,
                "turn_center_y": float(rng.uniform(-0.5, 0.5)),
                "turn_radius": radius,
                "turn_phase": float(rng.uniform(-np.pi, np.pi)),
                "turn_rate": turn_speed / radius,
                "x": x + radius,
                "y": float(rng.uniform(-0.5, 0.5)),
            })
        elif kind == "sudden_appearance":
            kwargs.update({
                "x": float(rng.uniform(8.0, route_length - 5.0)),
                "y": float(rng.uniform(-1.5, 1.5)),
                "vx": float(rng.uniform(-0.20, 0.20)),
                "vy": float(rng.uniform(-0.40, 0.40)),
                "appear_time": float(rng.uniform(8.0, 35.0)),
            })
        obstacles.append(ScenarioObstacle(**kwargs))

    extra_kinds = ("static", "crossing", "same_direction")
    for index in range(len(SCENARIO_KINDS), obstacle_count):
        kind = str(rng.choice(extra_kinds))
        obstacles.append(ScenarioObstacle(
            obstacle_id=f"obstacle_{index:02d}_{kind}",
            kind=kind,
            x=float(rng.uniform(4.0, route_length - 3.0)),
            y=float(rng.uniform(-2.0, 2.0)),
            half_length=float(rng.uniform(0.18, 0.35)),
            half_width=float(rng.uniform(0.18, 0.35)),
            vx=float(rng.uniform(-0.2, 0.5)) if kind != "static" else 0.0,
            vy=float(rng.uniform(-0.5, 0.5)) if kind == "crossing" else 0.0,
            max_speed=max_obstacle_speed,
        ))

    return LongRouteScenario(
        route_length=float(route_length),
        max_obstacle_speed=float(max_obstacle_speed),
        obstacles=tuple(obstacles),
    )


def generate_multi_target_scenario(
    seed: int,
    route_length: float = 3.20,
    max_obstacle_speed: float = 1.5,
    obstacle_count: int = 3,
    irregular: bool = False,
    include_person: bool = False,
) -> LongRouteScenario:
    """Generate a compact, replayable scene with several simultaneous targets.

    This profile is intended for tracker/decision/PPO experiments rather than
    long-route coverage.  It puts separated static, crossing, and same-direction
    boxes in or near the robot's local planning corridor, so a single LiDAR scan
    can contain more than one cluster while still leaving a passing option.
    """
    if route_length <= 0.0:
        raise ValueError("route_length must be positive")
    if max_obstacle_speed <= 0.0 or max_obstacle_speed > 7.0:
        raise ValueError("max_obstacle_speed must be in (0, 7]")
    if obstacle_count < 2:
        raise ValueError("obstacle_count must be at least two")

    rng = np.random.default_rng(seed)
    # Preserve the compact historical layout for short experiments, but
    # distribute the three targets over long routes. Otherwise a nominal 10 m
    # task would put all obstacles in the first 2.35 m.
    long_route = route_length >= 5.0
    forward_min = max(0.85, min(1.15, route_length * 0.35))
    forward_max = max(forward_min + 0.20, min(route_length - 0.45, 2.35))
    if long_route:
        static_x_range = (0.16 * route_length, 0.28 * route_length)
        crossing_x_range = (0.36 * route_length, 0.50 * route_length)
        irregular_x_range = (0.62 * route_length, 0.76 * route_length)
    else:
        static_x_range = (1.50, forward_max)
        crossing_x_range = (forward_min, min(forward_max, 1.85))
        irregular_x_range = (forward_min, min(forward_max, 1.55))
    nominal_speed = min(max_obstacle_speed, 0.45)
    third_kind = "person" if include_person else ("irregular" if irregular else "same_direction")
    third_kwargs: dict[str, Any] = {
        "obstacle_id": "multi_irregular_02" if irregular else "multi_same_direction_02",
        "kind": third_kind,
        "x": float(rng.uniform(*irregular_x_range)),
        "y": float(rng.uniform(-0.75, -0.45)),
        "half_length": float(rng.uniform(0.18, 0.28)),
        "half_width": float(rng.uniform(0.18, 0.28)),
        "vx": float(rng.uniform(0.05, min(0.25, max_obstacle_speed))),
        "vy": float(rng.uniform(-0.06, 0.06)),
        "max_speed": max_obstacle_speed,
    }
    if irregular:
        third_kwargs.update({
            "vx": float(rng.uniform(0.04, min(0.18, max_obstacle_speed))),
            "vy": float(rng.uniform(-0.04, 0.04)),
            "irregular_lateral_amplitude": float(rng.uniform(0.20, 0.38)),
            "irregular_frequency": float(rng.uniform(0.55, 1.05)),
            "irregular_phase": float(rng.uniform(-np.pi, np.pi)),
            "irregular_secondary_amplitude": float(rng.uniform(0.05, 0.14)),
            "irregular_secondary_frequency": float(rng.uniform(1.20, 2.20)),
            "irregular_secondary_phase": float(rng.uniform(-np.pi, np.pi)),
        })
    if include_person:
        # A pedestrian enters from a lateral lane with a small non-linear
        # component. Its footprint is narrower than the box obstacles.
        person_y = float(rng.choice([-1.10, 1.10]))
        person_vy = 0.26 if person_y < 0.0 else -0.26
        third_kwargs.update({
            "obstacle_id": "multi_person_02",
            "kind": "person",
            "y": person_y,
            "half_length": 0.18,
            "half_width": 0.18,
            "vx": float(rng.uniform(-0.02, 0.05)),
            "vy": person_vy,
            "height": 0.85,
            "irregular_lateral_amplitude": float(rng.uniform(0.05, 0.12)),
            "irregular_frequency": float(rng.uniform(0.45, 0.80)),
            "irregular_phase": float(rng.uniform(-np.pi, np.pi)),
            "irregular_secondary_amplitude": float(rng.uniform(0.02, 0.06)),
            "irregular_secondary_frequency": float(rng.uniform(1.0, 1.8)),
            "irregular_secondary_phase": float(rng.uniform(-np.pi, np.pi)),
        })
    obstacles = [
        ScenarioObstacle(
            obstacle_id="multi_static_00",
            kind="static",
            x=float(rng.uniform(*static_x_range)),
            y=float(rng.uniform(0.55, 0.95)),
            half_length=float(rng.uniform(0.20, 0.32)),
            half_width=float(rng.uniform(0.20, 0.32)),
            max_speed=max_obstacle_speed,
        ),
        ScenarioObstacle(
            obstacle_id="multi_crossing_01",
            kind="crossing",
            x=float(rng.uniform(*crossing_x_range)),
            y=float(rng.choice([-1.15, 1.15])),
            half_length=float(rng.uniform(0.20, 0.30)),
            half_width=float(rng.uniform(0.20, 0.30)),
            vy=float(rng.choice([-nominal_speed, nominal_speed])),
            max_speed=max_obstacle_speed,
        ),
        ScenarioObstacle(**third_kwargs),
    ][:min(3, obstacle_count)]

    # Additional objects remain separated in lateral lanes and are useful for
    # stress-testing the track cap without changing the first three targets.
    for index in range(len(obstacles), obstacle_count):
        obstacles.append(ScenarioObstacle(
            obstacle_id=f"multi_extra_{index:02d}",
            kind="static",
            x=float(rng.uniform(forward_min, forward_max)),
            y=float(rng.choice([-1.35, 1.35])),
            half_length=float(rng.uniform(0.18, 0.28)),
            half_width=float(rng.uniform(0.18, 0.28)),
            max_speed=max_obstacle_speed,
        ))

    return LongRouteScenario(
        route_length=float(route_length),
        max_obstacle_speed=float(max_obstacle_speed),
        obstacles=tuple(obstacles),
        lidar_model="multi_target_2d_lidar",
        lidar_scan_rate_hz=10.0,
        lidar_max_range=5.0,
        lidar_angular_resolution_deg=1.5,
    )


RANDOM_MOTION_KINDS = (
    "static",
    "crossing",
    "same_direction",
    "accelerating",
    "turning",
    "irregular",
)


WAIT_YIELD_CURRICULUM_CASES = (
    "crossing_wait",
    "persistent_blocker",
    "safe_passage",
    "oncoming_no_wait",
    "near_goal_crossing",
)


def wait_yield_curriculum_case(index: int) -> str:
    """Return the deterministic 20-episode curriculum label.

    The 20-slot cycle is 7/5/4/2/2, corresponding to the requested
    35/25/20/10/10 distribution.  Keeping the assignment index-based makes
    both SB3 training and fixed-seed evaluation reproducible.
    """
    slot = int(index) % 20
    if slot < 7:
        return WAIT_YIELD_CURRICULUM_CASES[0]
    if slot < 12:
        return WAIT_YIELD_CURRICULUM_CASES[1]
    if slot < 16:
        return WAIT_YIELD_CURRICULUM_CASES[2]
    if slot < 18:
        return WAIT_YIELD_CURRICULUM_CASES[3]
    return WAIT_YIELD_CURRICULUM_CASES[4]


def generate_wait_yield_curriculum_scenario(
    seed: int,
    *,
    route_length: float = 10.0,
    curriculum_index: int = 0,
    max_obstacle_speed: float = 1.5,
) -> LongRouteScenario:
    """Generate a labelled scene for learning when to yield.

    Each scene contains two pedestrian proxies and one object proxy.  Only
    the labelled primary target is relevant to the curriculum case; the two
    other targets are distractors placed outside the straight global corridor.
    This keeps the observation format close to the later random
    two-people-plus-object fine-tuning distribution.
    """
    if route_length < 4.0:
        raise ValueError("WAIT_YIELD curriculum needs a route of at least 4 m")
    rng = np.random.default_rng(int(seed) ^ 0xA17E5EED)
    case = wait_yield_curriculum_case(curriculum_index)
    direction = -1.0 if rng.random() < 0.5 else 1.0
    primary_kind = "person"
    primary: dict[str, Any] = {
        "obstacle_id": f"curriculum_primary_{case}",
        "kind": primary_kind,
        "height": 0.85,
        "half_length": 0.18,
        "half_width": 0.18,
        "max_speed": max_obstacle_speed,
        "motion_kind": "person",
    }
    if case == "crossing_wait":
        # Block the path now, but clear it within roughly one second if the
        # robot yields.  The obstacle starts far enough ahead that waiting
        # does not create a stationary-robot collision risk.
        primary.update({
            "x": float(rng.uniform(0.78, 1.02)),
            "y": 0.0,
            "vx": 0.0,
            "vy": direction * float(rng.uniform(0.22, 0.30)),
        })
    elif case == "persistent_blocker":
        primary.update({
            "x": float(rng.uniform(1.75, 2.35)),
            "y": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "motion_kind": "static",
        })
    elif case == "safe_passage":
        primary.update({
            "x": float(rng.uniform(2.0, 4.0)),
            "y": float(rng.choice([-1.45, 1.45])),
            "vx": 0.0,
            "vy": 0.0,
            "motion_kind": "static",
        })
    elif case == "oncoming_no_wait":
        # An oncoming target remains a genuine collision threat if the robot
        # stops.  This is deliberately not a waitable crossing case.
        primary.update({
            "x": float(rng.uniform(0.78, 1.02)),
            "y": 0.0,
            "vx": -float(rng.uniform(0.38, 0.62)),
            "vy": 0.0,
            "motion_kind": "same_direction",
        })
    elif case == "near_goal_crossing":
        primary.update({
            "x": float(rng.uniform(route_length - 2.0, route_length - 1.0)),
            "y": 0.0,
            "vx": 0.0,
            "vy": direction * float(rng.uniform(0.22, 0.30)),
        })
    else:
        raise AssertionError(f"unsupported curriculum case: {case}")

    distractors = [
        ScenarioObstacle(
            obstacle_id="curriculum_person_distractor",
            kind="person",
            motion_kind="static",
            x=float(rng.uniform(0.42 * route_length, 0.62 * route_length)),
            y=float(rng.choice([-1.45, 1.45])),
            half_length=0.18,
            half_width=0.18,
            height=0.85,
            max_speed=max_obstacle_speed,
        ),
        ScenarioObstacle(
            obstacle_id="curriculum_object_distractor",
            kind="object",
            motion_kind="static",
            x=float(rng.uniform(0.68 * route_length, 0.86 * route_length)),
            y=float(rng.choice([-1.55, 1.55])),
            half_length=0.28,
            half_width=0.28,
            height=0.25,
            max_speed=max_obstacle_speed,
        ),
    ]
    primary_obstacle = ScenarioObstacle(**primary)
    return LongRouteScenario(
        route_length=float(route_length),
        max_obstacle_speed=float(max_obstacle_speed),
        obstacles=(primary_obstacle, *distractors),
        lidar_model="wait_yield_curriculum_2d_lidar",
        lidar_scan_rate_hz=10.0,
        lidar_max_range=5.0,
        lidar_angular_resolution_deg=1.5,
    )


def generate_people_random_multi_target_scenario(
    seed: int,
    route_length: float = 10.0,
    max_obstacle_speed: float = 1.5,
    *,
    motion_schedule: tuple[str, str, str] | None = None,
    mirror_sign: float | None = None,
) -> LongRouteScenario:
    """Generate exactly two pedestrians and one object with random motion.

    The physical type is intentionally independent from the motion family:
    each pedestrian may be static, crossing, turning, or irregular, while the
    third target is a box proxy with its own independently sampled behavior.
    The seed fixes both the target placement and all motion parameters.
    """
    if route_length <= 0.0:
        raise ValueError("route_length must be positive")
    if max_obstacle_speed <= 0.0 or max_obstacle_speed > 7.0:
        raise ValueError("max_obstacle_speed must be in (0, 7]")

    rng = np.random.default_rng(seed)

    if motion_schedule is not None and len(motion_schedule) != 3:
        raise ValueError("motion_schedule must contain exactly three entries")
    if motion_schedule is not None and any(
        motion not in RANDOM_MOTION_KINDS for motion in motion_schedule
    ):
        raise ValueError("motion_schedule contains an unknown motion kind")

    def random_motion(
        x: float,
        y: float,
        forced_motion: str | None = None,
    ) -> dict[str, Any]:
        motion_kind = forced_motion or str(rng.choice(RANDOM_MOTION_KINDS))
        speed = float(rng.uniform(0.10, min(0.60, max_obstacle_speed)))
        values: dict[str, Any] = {"motion_kind": motion_kind}
        if motion_kind == "crossing":
            start_y = (
                float(np.copysign(1.20, y))
                if mirror_sign is not None
                else float(rng.choice([-1.20, 1.20]))
            )
            values.update({
                "y": start_y,
                "vx": float(rng.uniform(-0.08, 0.08)),
                "vy": speed if start_y < 0.0 else -speed,
            })
        elif motion_kind == "same_direction":
            values.update({
                "vx": float(rng.uniform(0.05, min(0.35, max_obstacle_speed))),
                "vy": float(rng.uniform(-0.12, 0.12)),
            })
        elif motion_kind == "accelerating":
            values.update({
                "vx": float(rng.uniform(0.02, min(0.30, max_obstacle_speed))),
                "vy": float(rng.uniform(-0.12, 0.12)),
                "ax": float(rng.uniform(0.02, min(0.12, max_obstacle_speed / 4.0))),
                "ay": float(rng.uniform(-0.05, 0.05)),
            })
        elif motion_kind == "turning":
            radius = float(rng.uniform(0.55, 1.20))
            phase = float(rng.uniform(-np.pi, np.pi))
            center_x = float(np.clip(x, 1.0, route_length - 1.0))
            center_y = float(rng.uniform(-0.45, 0.45))
            values.update({
                "turn_center_x": center_x,
                "turn_center_y": center_y,
                "turn_radius": radius,
                "turn_phase": phase,
                "turn_rate": speed / radius,
                "x": center_x + radius * np.cos(phase),
                "y": center_y + radius * np.sin(phase),
            })
        elif motion_kind == "irregular":
            values.update({
                "vx": float(rng.uniform(-0.04, 0.22)),
                "vy": float(rng.uniform(-0.10, 0.10)),
                "irregular_lateral_amplitude": float(rng.uniform(0.08, 0.32)),
                "irregular_frequency": float(rng.uniform(0.35, 1.10)),
                "irregular_phase": float(rng.uniform(-np.pi, np.pi)),
                "irregular_secondary_amplitude": float(rng.uniform(0.02, 0.12)),
                "irregular_secondary_frequency": float(rng.uniform(1.0, 2.4)),
                "irregular_secondary_phase": float(rng.uniform(-np.pi, np.pi)),
            })
        else:
            values.update({"vx": 0.0, "vy": 0.0})
        return values

    targets: list[ScenarioObstacle] = []
    for index, (kind, x_fraction, height) in enumerate((
        ("person", (0.18, 0.36), 0.85),
        ("person", (0.48, 0.66), 0.85),
        ("object", (0.68, 0.84), 0.25),
    )):
        x = float(rng.uniform(route_length * x_fraction[0], route_length * x_fraction[1]))
        if mirror_sign is None:
            y = float(rng.uniform(-0.95, 0.95))
        else:
            side = float(np.copysign(1.0, mirror_sign))
            if index % 2:
                side *= -1.0
            y = side * float(rng.uniform(0.20, 0.95))
        dimensions = (0.18, 0.18) if kind == "person" else (
            float(rng.uniform(0.20, 0.34)),
            float(rng.uniform(0.20, 0.34)),
        )
        forced_motion = None if motion_schedule is None else motion_schedule[index]
        motion = random_motion(x, y, forced_motion)
        target_kwargs: dict[str, Any] = {
            "obstacle_id": f"random_{kind}_{index:02d}",
            "kind": kind,
            "x": x,
            "y": y,
            "half_length": dimensions[0],
            "half_width": dimensions[1],
            "height": height,
            "max_speed": max_obstacle_speed,
            **motion,
        }
        targets.append(ScenarioObstacle(**target_kwargs))

    return LongRouteScenario(
        route_length=float(route_length),
        max_obstacle_speed=float(max_obstacle_speed),
        obstacles=tuple(targets),
        lidar_model="random_people_object_2d_lidar",
        lidar_scan_rate_hz=10.0,
        lidar_max_range=5.0,
        lidar_angular_resolution_deg=1.5,
    )


def generate_people_balanced_multi_target_scenario(
    seed: int,
    route_length: float = 10.0,
    max_obstacle_speed: float = 1.5,
    curriculum_index: int = 0,
) -> LongRouteScenario:
    """Generate a stratified, mirrored two-person/one-object episode.

    Across six consecutive episodes every physical target receives each of
    the six motion families exactly once.  The lateral layout is mirrored on
    the following six episodes, preventing a persistent left/right action
    bias while retaining random speeds, dimensions, and longitudinal poses.
    """
    index = max(int(curriculum_index), 0)
    schedule = tuple(
        RANDOM_MOTION_KINDS[(index + offset) % len(RANDOM_MOTION_KINDS)]
        for offset in range(3)
    )
    mirror_sign = 1.0 if (index // len(RANDOM_MOTION_KINDS)) % 2 == 0 else -1.0
    return generate_people_random_multi_target_scenario(
        seed,
        route_length=route_length,
        max_obstacle_speed=max_obstacle_speed,
        motion_schedule=schedule,
        mirror_sign=mirror_sign,
    )
