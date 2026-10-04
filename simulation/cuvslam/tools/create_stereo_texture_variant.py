#!/usr/bin/env python3
"""Add deterministic, non-repeating near-field stereo texture to an SDF."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


MODEL_NAME = "near_field_observability_boards"

# (board, lateral center, height center, lateral size, height size, gray)
PATCHES = (
    ("left", -1.135, 0.18, 0.075, 0.060, 0.96),
    ("left", -1.025, 0.29, 0.050, 0.095, 0.08),
    ("left", -0.895, 0.16, 0.090, 0.045, 0.72),
    ("left", -0.760, 0.28, 0.055, 0.070, 0.12),
    ("left", -0.625, 0.17, 0.065, 0.085, 0.91),
    ("left", -0.490, 0.31, 0.095, 0.050, 0.32),
    ("left", -1.145, 0.49, 0.060, 0.105, 0.18),
    ("left", -1.010, 0.56, 0.105, 0.045, 0.83),
    ("left", -0.865, 0.46, 0.045, 0.075, 0.05),
    ("left", -0.720, 0.58, 0.080, 0.090, 0.97),
    ("left", -0.565, 0.48, 0.055, 0.055, 0.46),
    ("left", -0.420, 0.62, 0.100, 0.060, 0.06),
    ("left", -1.110, 0.80, 0.085, 0.050, 0.67),
    ("left", -0.955, 0.88, 0.055, 0.100, 0.04),
    ("left", -0.805, 0.77, 0.110, 0.055, 0.94),
    ("left", -0.650, 0.90, 0.050, 0.070, 0.24),
    ("left", -0.505, 0.79, 0.070, 0.095, 0.87),
    ("left", -0.365, 0.91, 0.045, 0.045, 0.10),
    ("right", -3.035, 0.20, 0.060, 0.090, 0.94),
    ("right", -2.905, 0.32, 0.100, 0.050, 0.07),
    ("right", -2.760, 0.18, 0.050, 0.070, 0.58),
    ("right", -2.620, 0.29, 0.075, 0.100, 0.97),
    ("right", -2.475, 0.17, 0.095, 0.045, 0.16),
    ("right", -2.335, 0.34, 0.045, 0.080, 0.78),
    ("right", -3.010, 0.51, 0.085, 0.055, 0.11),
    ("right", -2.870, 0.59, 0.050, 0.105, 0.89),
    ("right", -2.725, 0.47, 0.110, 0.050, 0.03),
    ("right", -2.575, 0.61, 0.055, 0.075, 0.64),
    ("right", -2.430, 0.49, 0.075, 0.090, 0.96),
    ("right", -2.300, 0.63, 0.045, 0.045, 0.21),
    ("right", -3.045, 0.82, 0.055, 0.070, 0.73),
    ("right", -2.910, 0.91, 0.095, 0.050, 0.05),
    ("right", -2.770, 0.79, 0.045, 0.095, 0.92),
    ("right", -2.625, 0.89, 0.080, 0.055, 0.35),
    ("right", -2.475, 0.78, 0.105, 0.070, 0.98),
    ("right", -2.325, 0.92, 0.050, 0.085, 0.09),
)


def add_box_patch(
    link: ET.Element,
    name: str,
    pose: str,
    size: str,
    gray: float,
) -> None:
    visual = ET.SubElement(link, "visual", {"name": name})
    ET.SubElement(visual, "pose").text = pose
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = size
    material = ET.SubElement(visual, "material")
    color = f"{gray:g} {gray:g} {gray:g} 1"
    ET.SubElement(material, "ambient").text = color
    ET.SubElement(material, "diffuse").text = color


def add_near_field_texture(source_path: str, output_path: str) -> int:
    tree = ET.parse(source_path)
    models = {
        model.get("name"): model
        for model in tree.getroot().findall(".//model")
    }
    model = models.get(MODEL_NAME)
    if model is None:
        raise ValueError(f"Missing model: {MODEL_NAME}")
    link = model.find("link")
    if link is None:
        raise ValueError(f"Model {MODEL_NAME} has no link")
    if link.find("./visual[@name='stereo_texture_00']") is not None:
        raise ValueError("Stereo texture already exists")

    for index, (board, lateral, height, lateral_size, height_size, gray) in enumerate(
        PATCHES
    ):
        if board == "left":
            pose = f"-0.329 {lateral:g} {height:g} 0 0 0"
        else:
            pose = f"0.171 {lateral:g} {height:g} 0 0 0"
        add_box_patch(
            link,
            f"stereo_texture_{index:02d}",
            pose,
            f"0.006 {lateral_size:g} {height_size:g}",
            gray,
        )

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return len(PATCHES)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source")
    parser.add_argument("output")
    args = parser.parse_args()
    count = add_near_field_texture(args.source, args.output)
    print(f"created={Path(args.output)} stereo_texture_patches={count}")


if __name__ == "__main__":
    main()
