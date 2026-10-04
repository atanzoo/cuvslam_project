#!/usr/bin/env python3
"""Fail-closed command contract for the isolated Map-3 Nav2/MuJoCo bridge.

No ROS or hardware imports. The bridge is the sole simulated model-command
owner; this code must never be connected to robot drivers.
"""

from __future__ import annotations

import math
import threading
import time


ZERO_COMMAND = (0.0, 0.0, 0.0)


class SimCommandGuard:
    """Guard a Nav2 Twist before forwarding it to a MuJoCo-only model."""

    def __init__(
        self,
        *,
        scene_id: str,
        map_bounds: tuple[float, float, float, float],
        pose_timeout_s: float = 0.30,
        command_timeout_s: float = 0.30,
        max_vx: float = 0.30,
        max_vy: float = 0.02,
        max_wz: float = 0.70,
        clock=time.monotonic,
    ) -> None:
        if not scene_id or not all(math.isfinite(value) for value in map_bounds):
            raise ValueError("scene identity and finite map bounds are required")
        x_min, x_max, y_min, y_max = map_bounds
        if not x_min < x_max or not y_min < y_max:
            raise ValueError("map bounds are invalid")
        if min(pose_timeout_s, command_timeout_s, max_vx, max_vy, max_wz) <= 0:
            raise ValueError("positive timeout and speed limits are required")
        self.scene_id = scene_id
        self.map_bounds = map_bounds
        self.pose_timeout_s = float(pose_timeout_s)
        self.command_timeout_s = float(command_timeout_s)
        self.speed_limits = (float(max_vx), float(max_vy), float(max_wz))
        self.clock = clock
        self.lock = threading.Lock()
        self.pose_sequence = -1
        self.pose_time: float | None = None
        self.pose: tuple[float, float, float, float, float, float] | None = None
        self.command_time: float | None = None
        self.arm_time: float | None = None
        self.command = ZERO_COMMAND
        self.armed = False
        self.fault_reason: str | None = None

    def accept_pose(self, message: dict) -> tuple[float, float, float, float, float, float]:
        if message.get("scene_id") != self.scene_id:
            raise ValueError("pose scene identity mismatch")
        sequence = message.get("sequence")
        if type(sequence) is not int or sequence < 0:
            raise ValueError("pose sequence must be a nonnegative integer")
        fields = ("x", "y", "yaw", "vx", "vy", "wz")
        try:
            values = tuple(float(message[field]) for field in fields)
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("pose needs six numeric fields") from error
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose values must be finite")
        x, y = values[:2]
        x_min, x_max, y_min, y_max = self.map_bounds
        if not (x_min <= x < x_max and y_min <= y < y_max):
            raise ValueError("pose is outside the simulation map")
        with self.lock:
            if sequence <= self.pose_sequence:
                raise ValueError("pose sequence must increase")
            self.pose_sequence = sequence
            self.pose_time = self.clock()
            self.pose = values
        return values

    def arm(self, scene_id: str) -> None:
        if scene_id != self.scene_id:
            raise ValueError("arm scene identity mismatch")
        with self.lock:
            now = self.clock()
            if self.pose_time is None or now - self.pose_time > self.pose_timeout_s:
                raise RuntimeError("cannot arm without fresh simulation pose")
            self.armed = True
            self.arm_time = now
            self.fault_reason = None
            self.command = ZERO_COMMAND
            self.command_time = None

    def pose_snapshot(self) -> tuple[int, tuple[float, float, float, float, float, float]] | None:
        """Return only a fresh pose for TF/odom publication."""
        with self.lock:
            if self.pose is None or self.pose_time is None:
                return None
            if self.clock() - self.pose_time > self.pose_timeout_s:
                return None
            return self.pose_sequence, self.pose

    def disarm(self, reason: str = "explicit_disarm") -> None:
        with self.lock:
            self.armed = False
            self.arm_time = None
            self.fault_reason = reason
            self.command = ZERO_COMMAND
            self.command_time = None

    def accept_command(self, vx: float, vy: float, wz: float) -> None:
        values = (float(vx), float(vy), float(wz))
        with self.lock:
            if not self.armed:
                return
            if not all(math.isfinite(value) for value in values) or any(
                abs(value) > limit + 1e-9
                for value, limit in zip(values, self.speed_limits)
            ):
                self.armed = False
                self.arm_time = None
                self.fault_reason = "invalid_or_overspeed_nav2_command"
                self.command = ZERO_COMMAND
                self.command_time = None
                return
            self.command = values
            self.command_time = self.clock()

    def read_command(self) -> dict[str, object]:
        with self.lock:
            now = self.clock()
            if self.armed and (self.pose_time is None or now - self.pose_time > self.pose_timeout_s):
                self.armed = False
                self.fault_reason = "stale_simulation_pose"
            command_age_start = self.command_time if self.command_time is not None else self.arm_time
            if self.armed and (
                command_age_start is None
                or now - command_age_start > self.command_timeout_s
            ):
                self.armed = False
                self.fault_reason = "stale_nav2_command"
            if not self.armed:
                self.arm_time = None
                self.command = ZERO_COMMAND
                self.command_time = None
            return {
                "scene_id": self.scene_id,
                "armed": self.armed,
                "fault_reason": self.fault_reason,
                "last_pose_sequence": self.pose_sequence,
                "velocity_body": list(self.command),
            }
