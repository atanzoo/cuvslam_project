#!/usr/bin/env python3
"""Read-only candidate OccupancyGrid map support for offline D1 experiments.

The source PGM/YAML are read-only. ``scene_scale`` changes simulation geometry;
``preserve_resolution=True`` replicates integer-scale source cells in memory so
large scenes keep the original cell size and remain traceable to source hashes.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy import ndimage


FREE_PIXEL = 254
OCCUPIED_PIXEL = 0
UNKNOWN_PIXEL = 205


@dataclass(frozen=True)
class MapMetadata:
    image: str
    mode: str
    resolution: float
    origin: tuple[float, float, float]
    negate: int
    occupied_thresh: float
    free_thresh: float


def _next_pgm_token(stream) -> bytes:
    while True:
        first = stream.read(1)
        if not first:
            raise ValueError("unexpected end of PGM header")
        if first == b"#":
            stream.readline()
            continue
        if not first.isspace():
            token = bytearray(first)
            while True:
                char = stream.read(1)
                if not char or char.isspace():
                    break
                token.extend(char)
            return bytes(token)


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    with path.open("rb") as stream:
        magic = _next_pgm_token(stream)
        if magic not in {b"P5", b"P2"}:
            raise ValueError(f"unsupported PGM format: {magic!r}")
        width = int(_next_pgm_token(stream))
        height = int(_next_pgm_token(stream))
        max_value = int(_next_pgm_token(stream))
        if width <= 0 or height <= 0 or max_value != 255:
            raise ValueError("candidate PGM must have positive dimensions and max value 255")
        if magic == b"P5":
            pixels = stream.read(width * height)
        else:
            pixels = bytes(int(_next_pgm_token(stream)) for _ in range(width * height))
        expected = width * height
        if len(pixels) != expected:
            raise ValueError(f"PGM payload length {len(pixels)} != {expected}")
        return width, height, pixels


def _parse_float_list(value: str) -> tuple[float, ...]:
    stripped = value.strip().strip("[]")
    return tuple(float(part.strip()) for part in stripped.split(",") if part.strip())


def _read_yaml(path: Path) -> MapMetadata:
    values: dict[str, str] = {}
    lines = path.read_text(encoding="utf-8").splitlines()
    index = 0
    while index < len(lines):
        line = lines[index].strip()
        index += 1
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if key == "origin" and not value:
            origin_values: list[str] = []
            while index < len(lines) and len(origin_values) < 3:
                candidate = lines[index].strip()
                if not candidate.startswith("-"):
                    break
                origin_values.append(candidate[1:].strip())
                index += 1
            value = ",".join(origin_values)
        values[key] = value.strip('"\'')

    required = {
        "image", "mode", "resolution", "origin", "negate",
        "occupied_thresh", "free_thresh",
    }
    missing = sorted(required - values.keys())
    if missing:
        raise ValueError(f"map YAML missing fields: {', '.join(missing)}")
    origin = _parse_float_list(values["origin"])
    if len(origin) != 3:
        raise ValueError("map origin must contain x, y and yaw")
    metadata = MapMetadata(
        image=values["image"],
        mode=values["mode"],
        resolution=float(values["resolution"]),
        origin=(origin[0], origin[1], origin[2]),
        negate=int(values["negate"]),
        occupied_thresh=float(values["occupied_thresh"]),
        free_thresh=float(values["free_thresh"]),
    )
    if metadata.mode != "trinary":
        raise ValueError(f"only trinary candidate maps are supported, got {metadata.mode!r}")
    if metadata.resolution <= 0.0:
        raise ValueError("map resolution must be positive")
    if not 0.0 <= metadata.free_thresh < metadata.occupied_thresh <= 1.0:
        raise ValueError("map thresholds must satisfy 0 <= free < occupied <= 1")
    return metadata


@dataclass(frozen=True)
class CandidateMap:
    yaml_path: Path
    pgm_path: Path
    metadata: MapMetadata
    width: int
    height: int
    pixels: bytes
    scene_scale: float = 1.0
    preserve_resolution: bool = False
    source_width: int = 0
    source_height: int = 0

    @property
    def scaled_resolution(self) -> float:
        return self.metadata.resolution if self.preserve_resolution else self.metadata.resolution * self.scene_scale

    @property
    def scaled_origin(self) -> tuple[float, float, float]:
        x, y, yaw = self.metadata.origin
        return (x * self.scene_scale, y * self.scene_scale, yaw)

    @property
    def world_extent(self) -> tuple[float, float, float, float]:
        origin_x, origin_y, _ = self.scaled_origin
        return (
            origin_x,
            origin_y,
            origin_x + self.width * self.scaled_resolution,
            origin_y + self.height * self.scaled_resolution,
        )

    def pixel(self, column: int, row: int) -> int:
        if not (0 <= column < self.width and 0 <= row < self.height):
            raise IndexError("map cell is outside image bounds")
        return self.pixels[row * self.width + column]

    def classify_pixel(self, value: int) -> str:
        if value == UNKNOWN_PIXEL:
            return "unknown"
        occupancy = value / 255.0 if self.metadata.negate else 1.0 - value / 255.0
        if occupancy >= self.metadata.occupied_thresh:
            return "occupied"
        if occupancy <= self.metadata.free_thresh:
            return "free"
        return "unknown"

    def cell_to_world(self, column: int, row: int) -> tuple[float, float]:
        pixel = self.pixel(column, row)
        del pixel  # Keep this method usable for unknown and occupied cells too.
        origin_x, origin_y, _ = self.scaled_origin
        return (
            origin_x + (column + 0.5) * self.scaled_resolution,
            origin_y + (self.height - row - 0.5) * self.scaled_resolution,
        )

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("world coordinates must be finite")
        origin_x, origin_y, _ = self.scaled_origin
        column = math.floor((x - origin_x) / self.scaled_resolution)
        row_from_bottom = math.floor((y - origin_y) / self.scaled_resolution)
        row = self.height - 1 - row_from_bottom
        if not (0 <= column < self.width and 0 <= row < self.height):
            raise IndexError("world coordinate is outside map bounds")
        return column, row

    def counts(self) -> Counter[str]:
        return Counter(self.classify_pixel(value) for value in self.pixels)

    def known_free_cells(self) -> Iterable[tuple[int, int]]:
        for row in range(self.height):
            for column in range(self.width):
                if self.classify_pixel(self.pixel(column, row)) == "free":
                    yield column, row

    def largest_free_component(self) -> tuple[tuple[int, int], ...]:
        labels, count = self._known_free_component_labels()
        if count == 0:
            return ()
        sizes = np.bincount(labels.ravel())
        sizes[0] = 0
        coordinates = np.argwhere(labels == int(np.argmax(sizes)))
        return tuple((int(column), int(row)) for row, column in coordinates)

    def _known_free_component_labels(self) -> tuple[np.ndarray, int]:
        raw = np.frombuffer(self.pixels, dtype=np.uint8).reshape(self.height, self.width)
        occupancy = raw.astype(np.float32) / 255.0 if self.metadata.negate else 1.0 - raw.astype(np.float32) / 255.0
        free = (raw != UNKNOWN_PIXEL) & (occupancy <= self.metadata.free_thresh)
        return ndimage.label(
            free,
            structure=np.array(((0, 1, 0), (1, 1, 1), (0, 1, 0)), dtype=np.uint8),
        )

    def largest_known_free_component_size(self) -> int:
        labels, count = self._known_free_component_labels()
        if count == 0:
            return 0
        sizes = np.bincount(labels.ravel())
        return int(np.max(sizes[1:]))

    def manifest(self) -> dict[str, object]:
        counts = self.counts()
        return {
            "yaml_path": str(self.yaml_path),
            "pgm_path": str(self.pgm_path),
            "yaml_sha256": hashlib.sha256(self.yaml_path.read_bytes()).hexdigest(),
            "pgm_sha256": hashlib.sha256(self.pgm_path.read_bytes()).hexdigest(),
            "image_width": self.width,
            "image_height": self.height,
            "source_image_width": self.source_width or self.width,
            "source_image_height": self.source_height or self.height,
            "resolution": self.metadata.resolution,
            "origin": list(self.metadata.origin),
            "scene_scale": self.scene_scale,
            "preserve_resolution": self.preserve_resolution,
            "rasterization": "nearest_neighbor_cell_replication" if self.preserve_resolution else "source_grid",
            "scaled_resolution": self.scaled_resolution,
            "scaled_origin": list(self.scaled_origin),
            "pixel_counts": dict(counts),
            "source_pixel_counts": {
                key: value // int(round(self.scene_scale)) ** 2
                for key, value in counts.items()
            } if self.preserve_resolution else dict(counts),
            "largest_known_free_component_cells": self.largest_known_free_component_size(),
            "unknown_policy": "blocked",
        }


@dataclass(frozen=True)
class OccupiedRectangle:
    """Axis-aligned rectangle for a connected run of occupied map cells."""

    x: float
    y: float
    half_length: float
    half_width: float


def occupied_rectangles(candidate_map: CandidateMap) -> tuple[OccupiedRectangle, ...]:
    """Merge occupied pixels into axis-aligned rectangles for simulator walls.

    Unknown pixels are deliberately omitted here: they are blocked by the
    traversability mask, but must not be rendered as observed physical walls.
    The current map-to-MuJoCo adapter supports axis-aligned map origins only.
    """
    if not math.isclose(candidate_map.metadata.origin[2], 0.0, abs_tol=1e-9):
        raise ValueError("MuJoCo map walls require an axis-aligned map origin")
    raw = np.frombuffer(candidate_map.pixels, dtype=np.uint8).reshape(
        candidate_map.height, candidate_map.width,
    )
    occupancy = raw.astype(np.float32) / 255.0 if candidate_map.metadata.negate else (
        1.0 - raw.astype(np.float32) / 255.0
    )
    occupied = (raw != UNKNOWN_PIXEL) & (
        occupancy >= candidate_map.metadata.occupied_thresh
    )
    active: dict[tuple[int, int], int] = {}
    rectangles: list[tuple[int, int, int, int]] = []
    for row_index, row in enumerate(occupied):
        edges = np.flatnonzero(
            np.diff(np.r_[False, row, False].astype(np.int8))
        ).reshape(-1, 2)
        runs = {(int(start), int(end)) for start, end in edges}
        for interval in tuple(active):
            if interval not in runs:
                rectangles.append((interval[0], active.pop(interval), interval[1], row_index))
        for interval in runs:
            active.setdefault(interval, row_index)
    for interval, row_start in active.items():
        rectangles.append((interval[0], row_start, interval[1], candidate_map.height))

    origin_x, origin_y, _ = candidate_map.scaled_origin
    resolution = candidate_map.scaled_resolution
    result: list[OccupiedRectangle] = []
    for column_start, row_start, column_end, row_end in rectangles:
        x_min = origin_x + column_start * resolution
        x_max = origin_x + column_end * resolution
        y_min = origin_y + (candidate_map.height - row_end) * resolution
        y_max = origin_y + (candidate_map.height - row_start) * resolution
        result.append(OccupiedRectangle(
            x=(x_min + x_max) * 0.5,
            y=(y_min + y_max) * 0.5,
            half_length=(x_max - x_min) * 0.5,
            half_width=(y_max - y_min) * 0.5,
        ))
    return tuple(result)


def load_candidate_map(
    yaml_path: str | Path,
    *,
    scene_scale: float = 1.0,
    preserve_resolution: bool = False,
) -> CandidateMap:
    """Load a candidate map, optionally enlarging geometry at source resolution.

    With ``preserve_resolution=True``, ``scene_scale`` must be an integer and
    each source cell is replicated by that factor on both image axes. The
    source map files are never written.
    """
    yaml_file = Path(yaml_path).resolve()
    if not yaml_file.is_file():
        raise FileNotFoundError(yaml_file)
    if not math.isfinite(scene_scale) or scene_scale <= 0.0:
        raise ValueError("scene_scale must be finite and positive")
    metadata = _read_yaml(yaml_file)
    pgm_file = (yaml_file.parent / metadata.image).resolve()
    if not pgm_file.is_file():
        raise FileNotFoundError(pgm_file)
    source_width, source_height, source_pixels = _read_pgm(pgm_file)
    width, height, pixels = source_width, source_height, source_pixels
    if preserve_resolution:
        raster_scale = round(scene_scale)
        if not math.isclose(scene_scale, raster_scale, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("preserve_resolution requires an integer scene_scale")
        source_grid = np.frombuffer(source_pixels, dtype=np.uint8).reshape(source_height, source_width)
        pixels = np.repeat(np.repeat(source_grid, raster_scale, axis=0), raster_scale, axis=1).tobytes()
        width *= raster_scale
        height *= raster_scale
    return CandidateMap(
        yaml_path=yaml_file,
        pgm_path=pgm_file,
        metadata=metadata,
        width=width,
        height=height,
        pixels=pixels,
        scene_scale=float(scene_scale),
        preserve_resolution=bool(preserve_resolution),
        source_width=source_width,
        source_height=source_height,
    )
