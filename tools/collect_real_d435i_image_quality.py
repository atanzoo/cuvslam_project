#!/usr/bin/env python3
"""Stream lightweight IR-image quality metrics for a real D435i run.

The metrics are deliberately image-side diagnostics.  They do not alter the
camera stream or feed anything back into cuVSLAM.  This makes it possible to
compare two rooms while keeping IMU fusion and all estimator parameters fixed.
"""

from __future__ import annotations

import json
import math
import statistics
import time
from bisect import bisect_left
from collections import deque

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def percentile(values: list[float], percent: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def image_array(message: Image) -> np.ndarray | None:
    """Return a compact grayscale view without copying the whole frame."""
    encoding = message.encoding.lower()
    if encoding in {"mono8", "8uc1"}:
        row = np.frombuffer(message.data, dtype=np.uint8)
        if message.step < message.width:
            return None
        return row.reshape((message.height, message.step))[:, : message.width]
    if encoding in {"mono16", "16uc1"}:
        row = np.frombuffer(message.data, dtype=np.uint16)
        if message.step < message.width * 2:
            return None
        image = row.reshape((message.height, message.step // 2))[:, : message.width]
        return (image / 256.0).astype(np.uint8)
    return None


class StreamStats:
    def __init__(self) -> None:
        self.count = 0
        self.first_stamp_ns: int | None = None
        self.last_stamp_ns: int | None = None
        self.gaps_ms: deque[float] = deque(maxlen=240)
        self.metrics: deque[dict[str, float]] = deque(maxlen=120)
        self.previous: np.ndarray | None = None
        self.last_error = ""

    def add(self, message: Image, sample_stride: int) -> None:
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if self.last_stamp_ns is not None:
            self.gaps_ms.append((stamp_ns - self.last_stamp_ns) / 1e6)
        self.first_stamp_ns = self.first_stamp_ns or stamp_ns
        self.last_stamp_ns = stamp_ns
        self.count += 1
        if self.count % sample_stride:
            return
        image = image_array(message)
        if image is None:
            self.last_error = f"unsupported encoding={message.encoding}"
            return
        # Downsampling keeps this diagnostic from competing with cuVSLAM on
        # the Jetson while preserving room-scale brightness and texture cues.
        small = image[::4, ::4].astype(np.float32)
        flat = small.reshape(-1)
        dx = np.abs(np.diff(small, axis=1)).mean() if small.shape[1] > 1 else 0.0
        dy = np.abs(np.diff(small, axis=0)).mean() if small.shape[0] > 1 else 0.0
        frame_delta = 0.0
        if self.previous is not None and self.previous.shape == small.shape:
            frame_delta = float(np.abs(small - self.previous).mean())
        self.previous = small
        self.metrics.append(
            {
                "mean": float(flat.mean()),
                "std": float(flat.std()),
                "p01": float(np.percentile(flat, 1)),
                "p99": float(np.percentile(flat, 99)),
                "dark_pct": float(np.mean(flat <= 16.0) * 100.0),
                "bright_pct": float(np.mean(flat >= 245.0) * 100.0),
                "gradient": float((dx + dy) * 0.5),
                "frame_delta": frame_delta,
            }
        )

    def snapshot(self) -> dict[str, object]:
        values = list(self.metrics)
        latest = values[-1] if values else {}
        def vals(name: str) -> list[float]:
            return [float(item[name]) for item in values]
        rate_hz = 0.0
        if self.first_stamp_ns is not None and self.last_stamp_ns and self.last_stamp_ns > self.first_stamp_ns:
            rate_hz = (self.count - 1) / ((self.last_stamp_ns - self.first_stamp_ns) / 1e9)
        return {
            "count": self.count,
            "rate_hz": rate_hz,
            "gap_p95_ms": percentile(list(self.gaps_ms), 95.0),
            "gap_max_ms": max(self.gaps_ms, default=float("nan")),
            "latest": latest,
            "window": {
                "mean": statistics.fmean(vals("mean")) if values else float("nan"),
                "std": statistics.fmean(vals("std")) if values else float("nan"),
                "dark_pct": statistics.fmean(vals("dark_pct")) if values else float("nan"),
                "bright_pct": statistics.fmean(vals("bright_pct")) if values else float("nan"),
                "gradient": statistics.fmean(vals("gradient")) if values else float("nan"),
                "frame_delta": statistics.fmean(vals("frame_delta")) if values else float("nan"),
            },
            "error": self.last_error,
        }


def stamp_ns(message: Image) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def stereo_sync_snapshot(
    left_stamps: deque[int], right_stamps: deque[int]
) -> dict[str, float | int]:
    """Measure nearest left/right header-stamp offset without altering input."""
    if not left_stamps or not right_stamps:
        return {"samples": 0, "latest_us": float("nan"), "p95_us": float("nan"), "max_us": float("nan")}
    right_sorted = sorted(right_stamps)
    nearest_offsets_us: list[float] = []
    for left_stamp in left_stamps:
        index = bisect_left(right_sorted, left_stamp)
        candidates = []
        if index < len(right_sorted):
            candidates.append(right_sorted[index])
        if index:
            candidates.append(right_sorted[index - 1])
        if candidates:
            nearest_offsets_us.append(min(abs(left_stamp - value) for value in candidates) / 1e3)
    # A snapshot can be taken after the left callback but before the matching
    # right callback.  That creates one artificial ~33 ms edge sample at
    # 30 Hz.  Exclude these unpaired boundary frames while reporting their
    # ratio separately; genuine sub-frame stereo offsets remain measured.
    offsets_us = [value for value in nearest_offsets_us if value <= 20_000.0]
    unmatched = len(nearest_offsets_us) - len(offsets_us)
    return {
        "samples": len(offsets_us),
        "latest_us": offsets_us[-1] if offsets_us else float("nan"),
        "p95_us": percentile(offsets_us, 95.0),
        "max_us": max(offsets_us, default=float("nan")),
        "unmatched": unmatched,
        "unmatched_pct": (unmatched / len(nearest_offsets_us) * 100.0) if nearest_offsets_us else 0.0,
    }


def quality_flag(
    left: dict[str, object], right: dict[str, object], stereo_sync: dict[str, float | int]
) -> list[str]:
    issues: list[str] = []
    for label, stream in (("left", left), ("right", right)):
        window = stream.get("window", {})
        mean = float(window.get("mean", 0.0))
        std = float(window.get("std", 0.0))
        dark = float(window.get("dark_pct", 0.0))
        bright = float(window.get("bright_pct", 0.0))
        gradient = float(window.get("gradient", 0.0))
        if mean < 35.0 or dark > 45.0:
            issues.append(f"{label}:low_light")
        if mean > 220.0 or bright > 12.0:
            issues.append(f"{label}:clipped")
        if std < 12.0 or gradient < 2.0:
            issues.append(f"{label}:low_texture")
    if int(stereo_sync.get("samples", 0)) >= 3 and (
        float(stereo_sync.get("p95_us", 0.0)) > 100.0
        or float(stereo_sync.get("unmatched_pct", 0.0)) > 5.0
    ):
        issues.append("stereo:desync")
    return issues


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("real_d435i_image_quality_collector")
    qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    left = StreamStats()
    right = StreamStats()
    left_stamps: deque[int] = deque(maxlen=240)
    right_stamps: deque[int] = deque(maxlen=240)
    sample_stride = 3

    def add_left(message: Image) -> None:
        left_stamps.append(stamp_ns(message))
        left.add(message, sample_stride)

    def add_right(message: Image) -> None:
        right_stamps.append(stamp_ns(message))
        right.add(message, sample_stride)

    node.create_subscription(
        Image,
        "/camera/infra1/image_rect_raw",
        add_left,
        qos,
    )
    node.create_subscription(
        Image,
        "/camera/infra2/image_rect_raw",
        add_right,
        qos,
    )

    last_emit = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_emit < 0.5:
                continue
            last_emit = now
            left_snapshot = left.snapshot()
            right_snapshot = right.snapshot()
            stereo_sync = stereo_sync_snapshot(left_stamps, right_stamps)
            print(
                "LIVE_QUALITY "
                + json.dumps(
                    {
                        "left": left_snapshot,
                        "right": right_snapshot,
                        "stereo_sync": stereo_sync,
                        "flags": quality_flag(left_snapshot, right_snapshot, stereo_sync),
                    },
                    allow_nan=True,
                    separators=(",", ":"),
                ),
                flush=True,
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
