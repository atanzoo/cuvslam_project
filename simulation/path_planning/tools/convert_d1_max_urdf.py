#!/usr/bin/env python3
"""Prepare the official D1 Max URDF for local MuJoCo inspection.

The official URDF uses ROS ``package://`` mesh URIs and contains empty visual
material names.  ``urdf2mjcf`` does not resolve those two details on macOS, so
this tool performs only a deterministic path/material normalization after the
standard conversion.  It does not add gait control, contact tuning, or claim
that the resulting MJCF is a locomotion model.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET

import trimesh
from urdf2mjcf.convert import convert_urdf_to_mjcf


PACKAGE_MESH_PREFIX = "package://max_description/meshes/"
MUJOCO_MAX_MESH_FACES = 200_000


def _stl_face_count(path: Path) -> int:
    """Read the binary STL face count without changing the source mesh."""
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"STL is too small to inspect: {path}")
    return int(struct.unpack_from("<I", raw, 80)[0])


def _box_proxy_from_mesh(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return the official mesh bounds as a MuJoCo box center and half-size."""
    mesh = trimesh.load(path, force="mesh")
    bounds = mesh.bounds
    center = tuple(float(value) for value in ((bounds[0] + bounds[1]) / 2.0))
    half_size = tuple(float(value) for value in ((bounds[1] - bounds[0]) / 2.0))
    if any(value <= 0.0 for value in half_size):
        raise ValueError(f"D1 Max mesh has invalid bounds: {path}")
    return center, half_size


def prepare_d1_max_mjcf(urdf_path: str | Path, mjcf_path: str | Path) -> dict[str, int | str]:
    """Convert and normalize one official D1 Max URDF into a local MJCF."""
    source = Path(urdf_path)
    output = Path(mjcf_path)
    if not source.is_file():
        raise FileNotFoundError(f"D1 Max URDF not found: {source}")
    output.parent.mkdir(parents=True, exist_ok=True)
    raw_output = output.with_name(f"{output.stem}.raw.xml")

    convert_urdf_to_mjcf(source, raw_output, copy_meshes=False)
    tree = ET.parse(raw_output)
    root = tree.getroot()
    asset = root.find("asset")
    if asset is None:
        raise ValueError("converted D1 Max MJCF has no asset section")

    mesh_count = 0
    unresolved_meshes = []
    primitive_replacements: dict[str, dict[str, tuple[float, float, float] | int]] = {}
    for mesh in asset.findall("mesh"):
        mesh_file = mesh.attrib.get("file", "")
        if mesh_file.startswith(PACKAGE_MESH_PREFIX):
            mesh_file = "meshes/" + mesh_file.removeprefix(PACKAGE_MESH_PREFIX)
            mesh.attrib["file"] = mesh_file
        mesh_count += 1
        if not (output.parent / mesh_file).is_file():
            unresolved_meshes.append(mesh_file)
        source_mesh = source.parent.parent / "meshes" / Path(mesh_file).name
        if source_mesh.is_file() and _stl_face_count(source_mesh) > MUJOCO_MAX_MESH_FACES:
            center, half_size = _box_proxy_from_mesh(source_mesh)
            primitive_replacements[mesh.attrib["name"]] = {
                "faces": _stl_face_count(source_mesh),
                "center": center,
                "half_size": half_size,
            }
            asset.remove(mesh)

    for element in root.iter():
        mesh_name = element.attrib.get("mesh")
        replacement = primitive_replacements.get(mesh_name or "")
        if replacement is None:
            continue
        center = replacement["center"]
        half_size = replacement["half_size"]
        assert isinstance(center, tuple)
        assert isinstance(half_size, tuple)
        element.attrib.pop("mesh", None)
        element.attrib["type"] = "box"
        element.attrib["size"] = " ".join(f"{value:.9f}" for value in half_size)
        element.attrib["pos"] = " ".join(f"{value:.9f}" for value in center)

    empty_asset_materials = 0
    for material in list(asset.findall("material")):
        if not material.attrib.get("name", "").strip():
            asset.remove(material)
            empty_asset_materials += 1

    empty_geom_materials = 0
    for element in root.iter():
        if element.attrib.get("material") == "":
            element.attrib["material"] = "default_material"
            empty_geom_materials += 1

    if unresolved_meshes:
        raise FileNotFoundError(
            "D1 Max MJCF mesh files are missing: " + ", ".join(unresolved_meshes)
        )

    xml_text = ET.tostring(root, encoding="unicode")
    if "package://" in xml_text:
        raise ValueError("unresolved package:// URI remains in D1 Max MJCF")
    if re.search(r'<material\s+name=""', xml_text):
        raise ValueError("empty material name remains in D1 Max MJCF")

    tree.write(output, encoding="utf-8", xml_declaration=True)
    raw_output.unlink(missing_ok=True)
    return {
        "urdf": str(source),
        "mjcf": str(output),
        "mesh_count": mesh_count,
        "primitive_replacements": primitive_replacements,
        "empty_asset_materials_removed": empty_asset_materials,
        "empty_geom_materials_normalized": empty_geom_materials,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path("external/Agibot_D1_Max/urdf/max_description/urdf/max.urdf"),
    )
    parser.add_argument(
        "--mjcf",
        type=Path,
        default=Path("external/Agibot_D1_Max/d1_max.xml"),
    )
    args = parser.parse_args()
    result = prepare_d1_max_mjcf(args.urdf, args.mjcf)
    print(
        f"PASS D1 Max URDF converted: meshes={result['mesh_count']} "
        f"primitive_replacements={len(result['primitive_replacements'])} "
        f"asset_materials_removed={result['empty_asset_materials_removed']} "
        f"geom_materials_normalized={result['empty_geom_materials_normalized']}"
    )
    print(f"mjcf={result['mjcf']}")


if __name__ == "__main__":
    main()
