#!/usr/bin/env python3
"""Small simulation-side stand-in for the D1 Edu high-level SDK.

The real high-level interface accepts ``move(vx, vy, yaw_rate)`` and the
robot's internal controller turns that request into gait and motor commands.
This adapter keeps that boundary explicit in MuJoCo while approximating three
real-world effects: command saturation, command delay, and acceleration limits.
It intentionally does not pretend to be the official D1 controller.
"""

from __future__ import annotations

from collections import deque

import numpy as np


class SimulatedD1SDK:
    """Velocity-command interface with a simple actuator-side response model."""

    def __init__(
        self,
        max_velocity: np.ndarray | None = None,
        max_acceleration: np.ndarray | None = None,
        command_delay: float = 0.08,
    ) -> None:
        self.max_velocity = np.array(
            [0.36, 0.25, 0.60] if max_velocity is None else max_velocity,
            dtype=float,
        )
        self.max_acceleration = np.array(
            [0.60, 0.50, 1.50] if max_acceleration is None else max_acceleration,
            dtype=float,
        )
        if np.any(self.max_velocity <= 0.0):
            raise ValueError("max_velocity must be positive")
        if np.any(self.max_acceleration <= 0.0):
            raise ValueError("max_acceleration must be positive")
        if command_delay < 0.0:
            raise ValueError("command_delay cannot be negative")

        self.command_delay = float(command_delay)
        self.time = 0.0
        self.requested_command = np.zeros(3, dtype=float)
        self.applied_command = np.zeros(3, dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        self._pending: deque[tuple[float, np.ndarray]] = deque()

    def reset(self) -> None:
        self.time = 0.0
        self.requested_command[:] = 0.0
        self.applied_command[:] = 0.0
        self.velocity[:] = 0.0
        self._pending.clear()

    def move(self, command: np.ndarray, dt: float) -> np.ndarray:
        """Accept a local [vx, vy, yaw_rate] command and return actual velocity."""
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        requested = np.asarray(command, dtype=float).reshape(3)
        requested = np.clip(requested, -self.max_velocity, self.max_velocity)
        self.requested_command = requested.copy()
        self.time += float(dt)
        self._pending.append((self.time, requested.copy()))

        cutoff = self.time - self.command_delay
        while self._pending and self._pending[0][0] <= cutoff:
            _, delayed_command = self._pending.popleft()
            self.applied_command = delayed_command

        delta = self.applied_command - self.velocity
        step_limit = self.max_acceleration * dt
        self.velocity += np.clip(delta, -step_limit, step_limit)
        return self.velocity.copy()

    def getBodyVelocity(self) -> np.ndarray:
        """Mirror the SDK concept of body-frame velocity feedback."""
        return self.velocity.copy()

    def getWorldVelocity(self, yaw: float) -> np.ndarray:
        """Return planar world velocity for a supplied base yaw."""
        c, s = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c, -s], [s, c]])
        return np.concatenate((rotation @ self.velocity[:2], [self.velocity[2]]))
