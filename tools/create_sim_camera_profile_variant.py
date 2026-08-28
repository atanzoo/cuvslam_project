#!/usr/bin/env python3
"""Create an SDF variant with only the stereo image profile changed."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


CAMERA_SENSORS = ("d435i_infra1", "d435i_infra2")


def set_camera_image_profile(
    source_path: str,
    output_path: str,
    width: int,
    height: int,
) -> int:
    if width <= 0 or height <= 0:
        raise ValueError("Camera width and height must be positive")

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
        image = sensor.find("./camera/image")
        if image is None:
            raise ValueError(f"Sensor {name} has no camera/image element")
        width_element = image.find("width")
        height_element = image.find("height")
        if width_element is None or height_element is None:
            raise ValueError(f"Sensor {name} has incomplete image dimensions")
        width_element.text = str(width)
        height_element.text = str(height)
        changed += 1

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return changed


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source")
    parser.add_argument("output")
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    args = parser.parse_args()
    changed = set_camera_image_profile(
        args.source,
        args.output,
        args.width,
        args.height,
    )
    print(
        f"created={Path(args.output)} "
        f"camera_profile={args.width}x{args.height} "
        f"changed_sensors={changed}"
    )


if __name__ == "__main__":
    main()
