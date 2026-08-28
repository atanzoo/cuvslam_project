#!/usr/bin/env python3
"""Create a temporary SDF variant with only stereo camera rates changed."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


CAMERA_SENSORS = ("d435i_infra1", "d435i_infra2")


def set_camera_update_rate(
    source_path: str,
    output_path: str,
    update_rate_hz: float,
) -> int:
    if not math.isfinite(update_rate_hz) or update_rate_hz <= 0.0:
        raise ValueError("Camera update rate must be positive")
    tree = ET.parse(source_path)
    sensors = {
        sensor.get("name"): sensor
        for sensor in tree.getroot().findall(".//sensor")
    }
    changed = 0
    for name in CAMERA_SENSORS:
        sensor = sensors.get(name)
        if sensor is None:
            raise ValueError(f"Missing camera sensor: {name}")
        update_rate = sensor.find("update_rate")
        if update_rate is None:
            raise ValueError(f"Sensor {name} has no update_rate")
        update_rate.text = f"{update_rate_hz:g}"
        changed += 1
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return changed


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source")
    parser.add_argument("output")
    parser.add_argument("--rate", type=float, required=True)
    args = parser.parse_args()
    changed = set_camera_update_rate(args.source, args.output, args.rate)
    print(
        f"created={Path(args.output)} camera_rate_hz={args.rate:g} "
        f"changed_sensors={changed}"
    )


if __name__ == "__main__":
    main()
