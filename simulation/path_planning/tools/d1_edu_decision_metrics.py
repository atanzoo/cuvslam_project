#!/usr/bin/env python3
"""Post-episode metrics for learned high-level decision policies."""

from __future__ import annotations

from typing import Any, Iterable

import numpy as np


ACTION_NAMES = ("CRUISE", "AVOID_LEFT", "AVOID_RIGHT", "SLOWDOWN", "WAIT_YIELD")


def _rows_with_policy_actions(trace: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        row for row in trace
        if row.get("requested_action") is not None
        and row.get("robot") is not None
    ]


def summarize_decision_trace(
    trace: Iterable[dict[str, Any]],
    *,
    timeout: bool = False,
    distance_to_goal: float = float("inf"),
    goal_tolerance: float = 0.30,
    collision: bool = False,
) -> dict[str, Any]:
    """Summarize action quality without changing the simulator state.

    ``WAIT_YIELD`` quality is evaluated against the tracker estimate recorded
    at that decision update.  It is therefore a learning diagnostic, not a
    replacement for MuJoCo contact ground truth.
    """
    rows = _rows_with_policy_actions(trace)
    requested = [int(row["requested_action"]) for row in rows]
    counts = {name: 0 for name in ACTION_NAMES}
    for action in requested:
        if 0 <= action < len(ACTION_NAMES):
            counts[ACTION_NAMES[action]] += 1

    waitable_rows = [row for row in rows if float(row.get("waitable", 0.0)) > 0.5]
    valid_wait_rows = [
        row for row in waitable_rows
        if int(row["requested_action"]) == 4
        and float(row.get("stationary_collision_risk", 0.0)) < 0.5
    ]
    invalid_wait_rows = [
        row for row in rows
        if int(row["requested_action"]) == 4
        and row not in valid_wait_rows
    ]
    waitable_avoid_rows = [
        row for row in waitable_rows
        if int(row["requested_action"]) in (1, 2)
    ]

    wait_segments: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    for row in rows:
        if row.get("mode") == "WAIT_YIELD":
            current.append(row)
        elif current:
            wait_segments.append(current)
            current = []
    if current:
        wait_segments.append(current)

    wait_durations: list[float] = []
    wait_followed_progress = 0
    wait_returned_to_cruise = 0
    for segment in wait_segments:
        recorded_durations = [
            float(row.get("wait_duration", 0.0)) for row in segment
        ]
        duration = max(recorded_durations, default=0.0)
        if duration <= 0.0 and len(segment) >= 2:
            duration = max(
                0.0,
                float(segment[-1].get("t", 0.0))
                - float(segment[0].get("t", 0.0)),
            )
        wait_durations.append(duration)
        try:
            last_index = rows.index(segment[-1])
        except ValueError:
            continue
        following = rows[last_index + 1:last_index + 11]
        if not following:
            continue
        before_distance = float(np.linalg.norm(
            np.asarray(segment[-1]["robot"], dtype=float)
            - np.asarray(segment[-1].get("goal", [0.0, 0.0]), dtype=float)
        ))
        progressed = False
        returned = False
        for next_row in following:
            next_distance = float(np.linalg.norm(
                np.asarray(next_row["robot"], dtype=float)
                - np.asarray(next_row.get("goal", [0.0, 0.0]), dtype=float)
            ))
            progressed = progressed or next_distance < before_distance - 0.01
            returned = returned or next_row.get("mode") == "CRUISE"
        wait_followed_progress += int(progressed)
        wait_returned_to_cruise += int(returned)

    recent_rows = rows[-30:]
    if recent_rows:
        final_distance = float(distance_to_goal)
        near_goal_timeout = bool(
            timeout and final_distance <= max(goal_tolerance + 0.75, 1.0)
        )
        if len(recent_rows) >= 2:
            first_robot = np.asarray(recent_rows[0]["robot"], dtype=float)
            last_robot = np.asarray(recent_rows[-1]["robot"], dtype=float)
            recent_displacement = float(np.linalg.norm(last_robot - first_robot))
            recent_path = float(sum(
                np.linalg.norm(
                    np.asarray(a["robot"], dtype=float)
                    - np.asarray(b["robot"], dtype=float)
                )
                for a, b in zip(recent_rows, recent_rows[1:])
            ))
        else:
            recent_displacement = 0.0
            recent_path = 0.0
    else:
        near_goal_timeout = False
        recent_displacement = 0.0
        recent_path = 0.0
    real_deadlock = bool(
        timeout
        and not collision
        and not near_goal_timeout
        and distance_to_goal > max(goal_tolerance + 0.20, 0.60)
        and recent_displacement < 0.12
        and recent_path < 0.20
    )

    total = len(rows)
    waitable_count = len(waitable_rows)
    wait_requested_count = counts["WAIT_YIELD"]
    return {
        "decision_updates": total,
        "action_counts": counts,
        "action_proportions": {
            name: (count / total if total else 0.0)
            for name, count in counts.items()
        },
        "waitable_states": waitable_count,
        "wait_requested_count": wait_requested_count,
        "valid_wait_count": len(valid_wait_rows),
        "invalid_wait_count": len(invalid_wait_rows),
        "wait_argmax_rate": (
            len(valid_wait_rows) / waitable_count if waitable_count else 0.0
        ),
        "invalid_wait_rate": (
            len(invalid_wait_rows) / wait_requested_count
            if wait_requested_count else 0.0
        ),
        "waitable_avoid_count": len(waitable_avoid_rows),
        "waitable_avoid_rate": (
            len(waitable_avoid_rows) / waitable_count if waitable_count else 0.0
        ),
        "wait_segments": len(wait_segments),
        "mean_consecutive_wait_duration": (
            float(np.mean(wait_durations)) if wait_durations else 0.0
        ),
        "max_consecutive_wait_duration": (
            float(np.max(wait_durations)) if wait_durations else 0.0
        ),
        "wait_followed_progress_count": wait_followed_progress,
        "wait_returned_to_cruise_count": wait_returned_to_cruise,
        "near_goal_timeout": near_goal_timeout,
        "real_deadlock": real_deadlock,
        "recent_displacement": recent_displacement,
        "recent_path": recent_path,
    }
