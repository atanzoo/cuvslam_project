#!/usr/bin/env python3
"""Parse Gazebo Transport Pose_V text logs without losing entity metadata."""

from __future__ import annotations

import argparse
import csv
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class GzPoseSample:
    timestamp_ns: int
    name: str
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


def _balanced_blocks(text: str, field: str) -> list[str]:
    blocks: list[str] = []
    pattern = re.compile(rf"(?m)^\s*{re.escape(field)}\s*\{{")
    for match in pattern.finditer(text):
        depth = 0
        for index in range(match.end() - 1, len(text)):
            if text[index] == "{":
                depth += 1
            elif text[index] == "}":
                depth -= 1
                if depth == 0:
                    blocks.append(text[match.start():index + 1])
                    break
    return blocks


def _number(block: str, field: str, default: float = 0.0) -> float:
    match = re.search(
        rf"(?m)^\s*{re.escape(field)}:\s*"
        r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*$",
        block,
    )
    return float(match.group(1)) if match else default


def _named_block(block: str, field: str) -> str:
    values = _balanced_blocks(block, field)
    return values[0] if values else ""


def parse_pose_v_text(text: str, entity_name: str) -> list[GzPoseSample]:
    samples: list[GzPoseSample] = []
    message_starts = [
        match.start() for match in re.finditer(r"(?m)^header\s*\{", text)
    ]
    message_starts.append(len(text))

    for start, end in zip(message_starts, message_starts[1:]):
        message = text[start:end]
        stamp = _named_block(_named_block(message, "header"), "stamp")
        timestamp_ns = (
            int(_number(stamp, "sec")) * 1_000_000_000
            + int(_number(stamp, "nsec"))
        )
        for pose in _balanced_blocks(message, "pose"):
            name_match = re.search(r'(?m)^\s*name:\s*"([^"]+)"\s*$', pose)
            if not name_match or name_match.group(1) != entity_name:
                continue
            position = _named_block(pose, "position")
            orientation = _named_block(pose, "orientation")
            samples.append(
                GzPoseSample(
                    timestamp_ns=timestamp_ns,
                    name=entity_name,
                    translation=(
                        _number(position, "x"),
                        _number(position, "y"),
                        _number(position, "z"),
                    ),
                    rotation=(
                        _number(orientation, "x"),
                        _number(orientation, "y"),
                        _number(orientation, "z"),
                        _number(orientation, "w", 1.0),
                    ),
                )
            )
            break
    return samples


def read_pose_log(path: str, entity_name: str) -> list[GzPoseSample]:
    samples = parse_pose_v_text(
        Path(path).read_text(encoding="utf-8"),
        entity_name,
    )
    if len(samples) < 2:
        raise RuntimeError(
            f"{path} has fewer than two poses for entity {entity_name}"
        )
    return samples


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    parser.add_argument("--entity", default="slam_bot")
    parser.add_argument("--output")
    args = parser.parse_args()

    samples = read_pose_log(args.log, args.entity)
    output = open(args.output, "w", newline="", encoding="utf-8") \
        if args.output else None
    stream = output or __import__("sys").stdout
    try:
        writer = csv.writer(stream)
        writer.writerow(
            ("timestamp_ns", "name", "x", "y", "z", "qx", "qy", "qz", "qw")
        )
        for sample in samples:
            writer.writerow(
                (
                    sample.timestamp_ns,
                    sample.name,
                    *sample.translation,
                    *sample.rotation,
                )
            )
    finally:
        if output:
            output.close()


if __name__ == "__main__":
    main()
