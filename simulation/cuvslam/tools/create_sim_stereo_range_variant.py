#!/usr/bin/env python3
"""Create an SDF variant with a bounded stereo rendering range."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


CAMERA_SENSORS = ("d435i_infra1", "d435i_infra2")


def set_stereo_far_clip(
    source_path: str,
    output_path: str,
    far_clip_m: float,
) -> int:
    if not math.isfinite(far_clip_m) or far_clip_m <= 0.0:
        raise ValueError("Stereo far clip must be positive")

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
        near = sensor.find("./camera/clip/near")
        far = sensor.find("./camera/clip/far")
        if near is None or far is None:
            raise ValueError(f"Sensor {name} has incomplete clip limits")
        if far_clip_m <= float(near.text):
            raise ValueError("Stereo far clip must exceed near clip")
        far.text = f"{far_clip_m:g}"
        changed += 1

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return changed


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source")
    parser.add_argument("output")
    parser.add_argument("--far", type=float, required=True)
    args = parser.parse_args()
    changed = set_stereo_far_clip(args.source, args.output, args.far)
    print(
        f"created={Path(args.output)} stereo_far_clip_m={args.far:g} "
        f"changed_sensors={changed}"
    )


if __name__ == "__main__":
    main()
