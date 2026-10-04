#!/usr/bin/env python3
"""Read-only AMCL/map/scan telemetry collector and local browser monitor.

The state machine and rendering payload helpers are importable without ROS so
the contract tests remain useful on the Mac. With ROS 2 available, ``main``
subscribes to ``/map``, ``/amcl_pose`` and ``/scan`` and observes TF through
tf2. It never publishes a topic, TF, initial pose, service request, or command.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import threading
from collections import deque
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Optional


SCHEMA_VERSION = "experimental_localization_telemetry.v1"
STATE_WAIT = "WAIT"
STATE_OK = "OK"
STATE_REVIEW = "REVIEW"
STATE_LOST = "LOST"
SCAN_WAIT = "WAIT"
SCAN_OK = "OK"
SCAN_DELAY = "DELAY"
MAX_PENDING_SCANS = 16
MAX_RENDER_SCAN_POINTS = 1500
OVERLAP_NEAR_M = 0.15
OVERLAP_REVIEW_M = 0.30
ACTIVE_MAP_WARNING = (
    "Active /map is experimental; localization accuracy has not passed acceptance."
)


@dataclass
class LocalizationStateMachine:
    """Classify freshness and planar covariance without hiding missing data."""

    stale_after_s: float = 1.5
    lost_after_s: float = 3.0
    review_covariance_xy_m2: float = 0.25
    review_covariance_yaw_rad2: float = 0.25
    previous_state: str = STATE_WAIT

    def evaluate(
        self,
        *,
        now_s: float,
        pose_age_s: Optional[float],
        tf_age_s: Optional[float],
        covariance: Optional[dict[str, Optional[float]]],
        pose_frame_id: Optional[str] = None,
    ) -> tuple[str, list[str], Optional[str]]:
        del now_s
        reasons: list[str] = []
        if pose_age_s is None:
            reasons.append("amcl_pose_missing")
        if tf_age_s is None:
            reasons.append("map_to_odom_missing")

        oldest_age = max(
            [age for age in (pose_age_s, tf_age_s) if age is not None],
            default=None,
        )
        if pose_age_s is None and tf_age_s is None:
            state = STATE_WAIT
        elif pose_age_s is None or tf_age_s is None:
            state = STATE_REVIEW
            reasons.append("required_telemetry_missing")
        elif oldest_age is not None and oldest_age >= self.lost_after_s:
            state = STATE_LOST
            reasons.append("telemetry_stale")
        elif oldest_age >= self.stale_after_s:
            state = STATE_REVIEW
            reasons.append("telemetry_aging")
        elif pose_frame_id not in (None, "map"):
            state = STATE_REVIEW
            reasons.append("amcl_pose_frame_invalid")
        elif covariance is None:
            state = STATE_REVIEW
            reasons.append("planar_covariance_missing")
        else:
            xy = covariance.get("xy_m2")
            yaw = covariance.get("yaw_rad2")
            if xy is None or yaw is None:
                state = STATE_REVIEW
                reasons.append("planar_covariance_missing")
            elif xy > self.review_covariance_xy_m2:
                state = STATE_REVIEW
                reasons.append("xy_covariance_high")
            elif yaw > self.review_covariance_yaw_rad2:
                state = STATE_REVIEW
                reasons.append("yaw_covariance_high")
            else:
                state = STATE_OK

        transition = None
        if state != self.previous_state:
            transition = f"{self.previous_state}->{state}"
            if self.previous_state == STATE_LOST and state in {
                STATE_OK,
                STATE_REVIEW,
            }:
                reasons.append("recovered")
            self.previous_state = state
        return state, reasons, transition


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a ROS quaternion."""

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _age(now_s: float, stamp_s: Optional[float]) -> Optional[float]:
    if stamp_s is None:
        return None
    return max(0.0, now_s - stamp_s)


def _json_number(value: Optional[float]) -> Optional[float]:
    if value is None or not math.isfinite(value):
        return None
    return float(value)


def occupancy_to_display_bytes(data: Any) -> bytes:
    """Convert OccupancyGrid values to grayscale pixels for the read-only GUI."""

    pixels = bytearray()
    for value in data:
        numeric = int(value)
        if numeric < 0:
            pixels.append(135)  # unknown
        elif numeric >= 65:
            pixels.append(45)  # occupied
        elif numeric == 0:
            pixels.append(245)  # free
        else:
            pixels.append(175)  # intermediate/uncertain
    return bytes(pixels)


def transform_scan_to_map(
    ranges: Any,
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    transform_x: float,
    transform_y: float,
    transform_yaw: float,
    max_points: int = MAX_RENDER_SCAN_POINTS,
) -> tuple[list[dict[str, float]], int]:
    """Transform LaserScan points using one exact-time planar TF sample.

    The second return value is the number of valid input points before the
    bounded rendering sample is selected.
    """

    valid: list[tuple[float, float]] = []
    upper = range_max if range_max > 0.0 else float("inf")
    for index, distance in enumerate(ranges):
        distance = float(distance)
        if not math.isfinite(distance) or distance < range_min or distance > upper:
            continue
        angle = angle_min + index * angle_increment
        valid.append((distance * math.cos(angle), distance * math.sin(angle)))

    stride = max(1, math.ceil(len(valid) / max(1, max_points)))
    c = math.cos(transform_yaw)
    s = math.sin(transform_yaw)
    points = [
        {
            "x_m": transform_x + c * x - s * y,
            "y_m": transform_y + s * x + c * y,
        }
        for x, y in valid[::stride]
    ]
    return points, len(valid)


def classify_scan_overlap(
    points: list[dict[str, float]],
    map_snapshot: "MapSnapshot",
    *,
    near_m: float = OVERLAP_NEAR_M,
    review_m: float = OVERLAP_REVIEW_M,
) -> tuple[list[dict[str, float]], dict[str, Any]]:
    """Classify map-frame scan endpoints against occupied map cells.

    This is a visual alignment diagnostic only. It does not alter the map,
    scan, TF, AMCL, or pose estimate.
    """

    if not map_snapshot.available:
        return points, {
            "status": "WAIT_MAP",
            "point_count": len(points),
            "matched_near_count": 0,
            "matched_review_count": 0,
            "matched_near_ratio": None,
            "matched_review_ratio": None,
        }
    if not points:
        return points, {
            "status": "WAIT_SCAN",
            "point_count": 0,
            "matched_near_count": 0,
            "matched_review_count": 0,
            "matched_near_ratio": None,
            "matched_review_ratio": None,
        }

    classified: list[dict[str, float]] = []
    near_count = 0
    review_count = 0
    for point in points:
        distance_m = map_snapshot.nearest_occupied_distance(
            point["x_m"], point["y_m"], review_m
        )
        enriched = dict(point)
        if distance_m is None:
            enriched["overlap"] = "out_of_map"
            enriched["occupied_distance_m"] = None
        elif distance_m <= near_m:
            enriched["overlap"] = "near"
            enriched["occupied_distance_m"] = distance_m
            near_count += 1
            review_count += 1
        elif distance_m <= review_m:
            enriched["overlap"] = "review"
            enriched["occupied_distance_m"] = distance_m
            review_count += 1
        else:
            enriched["overlap"] = "unmatched"
            enriched["occupied_distance_m"] = distance_m
        classified.append(enriched)

    total = len(classified)
    return classified, {
        "status": "OBSERVE",
        "point_count": total,
        "matched_near_count": near_count,
        "matched_review_count": review_count,
        "matched_near_ratio": near_count / total,
        "matched_review_ratio": review_count / total,
        "near_threshold_m": near_m,
        "review_threshold_m": review_m,
        "note": "Alignment diagnostic only; not localization-accuracy acceptance.",
    }


@dataclass
class MapSnapshot:
    revision: int = 0
    available: bool = False
    width: int = 0
    height: int = 0
    resolution_m: float = 0.0
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0
    origin_yaw_rad: float = 0.0
    stamp_s: Optional[float] = None
    frame_id: str = "map"
    display_cells: bytes = b""
    occupied_cells: set[tuple[int, int]] = field(default_factory=set)

    def accept_grid(
        self,
        *,
        width: int,
        height: int,
        resolution_m: float,
        origin_x_m: float,
        origin_y_m: float,
        origin_yaw_rad: float,
        data: Any,
        stamp_s: Optional[float],
        frame_id: str = "map",
    ) -> None:
        expected = int(width) * int(height)
        if expected <= 0 or len(data) != expected:
            raise ValueError("occupancy grid dimensions do not match data length")
        self.revision += 1
        self.available = True
        self.width = int(width)
        self.height = int(height)
        self.resolution_m = float(resolution_m)
        self.origin_x_m = float(origin_x_m)
        self.origin_y_m = float(origin_y_m)
        self.origin_yaw_rad = float(origin_yaw_rad)
        self.stamp_s = stamp_s
        self.frame_id = frame_id or "map"
        self.display_cells = occupancy_to_display_bytes(data)
        self.occupied_cells = {
            (index % self.width, index // self.width)
            for index, value in enumerate(data)
            if int(value) >= 65
        }

    def nearest_occupied_distance(
        self, x_m: float, y_m: float, max_distance_m: float
    ) -> Optional[float]:
        if not self.available or not self.occupied_cells or self.resolution_m <= 0.0:
            return None
        c = math.cos(self.origin_yaw_rad)
        s = math.sin(self.origin_yaw_rad)
        dx = x_m - self.origin_x_m
        dy = y_m - self.origin_y_m
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        cell_x = math.floor(local_x / self.resolution_m)
        cell_y = math.floor(local_y / self.resolution_m)
        radius = math.ceil(max_distance_m / self.resolution_m)
        nearest = None
        for candidate_x in range(cell_x - radius, cell_x + radius + 1):
            for candidate_y in range(cell_y - radius, cell_y + radius + 1):
                if (candidate_x, candidate_y) not in self.occupied_cells:
                    continue
                center_x = (candidate_x + 0.5) * self.resolution_m
                center_y = (candidate_y + 0.5) * self.resolution_m
                distance = math.hypot(local_x - center_x, local_y - center_y)
                if nearest is None or distance < nearest:
                    nearest = distance
        return nearest if nearest is not None and nearest <= max_distance_m else None

    def metadata(self, now_s: Optional[float] = None) -> dict[str, Any]:
        return {
            "available": self.available,
            "topic": "/map",
            "revision": self.revision,
            "frame_id": self.frame_id,
            "width": self.width,
            "height": self.height,
            "resolution_m": _json_number(self.resolution_m),
            "origin": {
                "x_m": _json_number(self.origin_x_m),
                "y_m": _json_number(self.origin_y_m),
                "yaw_rad": _json_number(self.origin_yaw_rad),
            },
            "stamp_s": _json_number(self.stamp_s),
            "age_s": _json_number(_age(now_s, self.stamp_s)) if now_s is not None else None,
            "cell_count": len(self.display_cells),
        }

    def grid_payload(self) -> dict[str, Any]:
        return {
            "available": self.available,
            "revision": self.revision,
            "width": self.width,
            "height": self.height,
            "encoding": "gray_u8_base64",
            "data": base64.b64encode(self.display_cells).decode("ascii"),
        }


@dataclass
class ScanSnapshot:
    pending: deque[dict[str, Any]] = field(default_factory=deque)
    latest_input_stamp_s: Optional[float] = None
    latest_input_frame_id: str = ""
    display_points: list[dict[str, float]] = field(default_factory=list)
    display_stamp_s: Optional[float] = None
    display_tf_stamp_s: Optional[float] = None
    status: str = SCAN_WAIT
    valid_input_count: int = 0
    queue_drops: int = 0
    overlap: dict[str, Any] = field(
        default_factory=lambda: {"status": "WAIT_MAP", "point_count": 0}
    )

    def accept_scan(
        self,
        *,
        stamp_s: float,
        stamp_msg: Any,
        frame_id: str,
        ranges: Any,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> None:
        self.latest_input_stamp_s = stamp_s
        self.latest_input_frame_id = frame_id
        if len(self.pending) >= MAX_PENDING_SCANS:
            self.pending.popleft()
            self.queue_drops += 1
        self.pending.append(
            {
                "stamp_s": stamp_s,
                "stamp_msg": stamp_msg,
                "frame_id": frame_id,
                "ranges": tuple(ranges),
                "angle_min": float(angle_min),
                "angle_increment": float(angle_increment),
                "range_min": float(range_min),
                "range_max": float(range_max),
            }
        )

    def resolve(
        self,
        lookup: Callable[[dict[str, Any]], Optional[dict[str, float]]],
        now_s: float,
    ) -> None:
        if not self.pending:
            self.status = SCAN_DELAY if self.display_points else SCAN_WAIT
            return

        remaining: deque[dict[str, Any]] = deque()
        newest_resolved: Optional[tuple[dict[str, Any], dict[str, float]]] = None
        for entry in self.pending:
            result = lookup(entry)
            if result is None:
                remaining.append(entry)
            else:
                newest_resolved = (entry, result)
        self.pending = remaining

        if newest_resolved is None:
            self.status = SCAN_DELAY if self.display_points else SCAN_WAIT
            return

        entry, transform = newest_resolved
        points, valid_count = transform_scan_to_map(
            entry["ranges"],
            angle_min=entry["angle_min"],
            angle_increment=entry["angle_increment"],
            range_min=entry["range_min"],
            range_max=entry["range_max"],
            transform_x=transform["x_m"],
            transform_y=transform["y_m"],
            transform_yaw=transform["yaw_rad"],
        )
        self.display_points = points
        self.display_stamp_s = entry["stamp_s"]
        self.display_tf_stamp_s = transform.get("tf_stamp_s")
        self.valid_input_count = valid_count
        self.status = SCAN_OK
        del now_s

    def update_overlap(self, map_snapshot: MapSnapshot) -> None:
        self.display_points, self.overlap = classify_scan_overlap(
            self.display_points,
            map_snapshot,
        )

    def summary(self, now_s: float) -> dict[str, Any]:
        display_age = _age(now_s, self.display_stamp_s)
        return {
            "topic": "/scan",
            "status": self.status,
            "frame_id": self.latest_input_frame_id,
            "input_stamp_s": _json_number(self.latest_input_stamp_s),
            "input_age_s": _json_number(_age(now_s, self.latest_input_stamp_s)),
            "display_stamp_s": _json_number(self.display_stamp_s),
            "display_age_s": _json_number(display_age),
            "tf_stamp_s": _json_number(self.display_tf_stamp_s),
            "exact_timestamp_tf": self.status == SCAN_OK,
            "point_count": len(self.display_points),
            "valid_input_count": self.valid_input_count,
            "pending_scans": len(self.pending),
            "queue_drops": self.queue_drops,
            "retained_last_valid": bool(self.display_points) and self.status == SCAN_DELAY,
            "map_overlap": self.overlap,
        }


@dataclass
class MonitorSnapshot:
    state_machine: LocalizationStateMachine = field(
        default_factory=LocalizationStateMachine
    )
    map_snapshot: MapSnapshot = field(default_factory=MapSnapshot)
    scan_snapshot: ScanSnapshot = field(default_factory=ScanSnapshot)
    latest_pose: Optional[dict[str, Any]] = None
    latest_pose_stamp_s: Optional[float] = None
    latest_tf_stamp_s: Optional[float] = None
    latest_tf_available: bool = False
    trajectory: list[dict[str, float]] = field(default_factory=list)
    latest: dict[str, Any] = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update(
        self,
        *,
        now_s: float,
        tf_available: bool,
        tf_stamp_s: Optional[float],
    ) -> dict[str, Any]:
        with self.lock:
            self.latest_tf_available = tf_available
            self.latest_tf_stamp_s = tf_stamp_s
            self.scan_snapshot.update_overlap(self.map_snapshot)
            pose_age = _age(now_s, self.latest_pose_stamp_s)
            tf_age = _age(now_s, tf_stamp_s) if tf_available else None
            covariance = (
                self.latest_pose.get("covariance") if self.latest_pose else None
            )
            pose_frame_id = self.latest_pose.get("frame_id") if self.latest_pose else None
            state, reasons, transition = self.state_machine.evaluate(
                now_s=now_s,
                pose_age_s=pose_age,
                tf_age_s=tf_age,
                covariance=covariance,
                pose_frame_id=pose_frame_id,
            )
            self.latest = {
                "schema_version": SCHEMA_VERSION,
                "generated_at_s": now_s,
                "state": state,
                "state_transition": transition,
                "reasons": reasons,
                "pose": self.latest_pose,
                "pose_age_s": _json_number(pose_age),
                "map_to_odom": {
                    "available": bool(tf_available),
                    "age_s": _json_number(tf_age),
                    "parent_frame": "map",
                    "child_frame": "odom",
                },
                "map": self.map_snapshot.metadata(now_s),
                "scan": self.scan_snapshot.summary(now_s),
                "map_overlap": self.scan_snapshot.overlap,
                "trajectory": list(self.trajectory[-100:]),
                "trajectory_count": len(self.trajectory),
                "read_only": True,
                "active_map_warning": ACTIVE_MAP_WARNING,
                "extrinsic_candidate": {
                    "parent_frame": "base_link",
                    "child_frame": "rplidar_link",
                    "x_m": 0.0,
                    "y_m": -0.10,
                    "z_m": 0.0,
                    "yaw_rad": 2.334,
                    "status": "candidate_unverified",
                },
            }
            return dict(self.latest)

    def render_payload(self, now_s: float) -> dict[str, Any]:
        with self.lock:
            return {
                "schema_version": SCHEMA_VERSION,
                "map": self.map_snapshot.metadata(now_s),
                "pose": self.latest_pose,
                "trajectory": list(self.trajectory),
                "scan": self.scan_snapshot.summary(now_s),
                "map_overlap": self.scan_snapshot.overlap,
                "scan_points": list(self.scan_snapshot.display_points),
                "read_only": True,
            }

    def accept_pose(self, pose: dict[str, Any], stamp_s: float) -> None:
        with self.lock:
            self.latest_pose = pose
            self.latest_pose_stamp_s = stamp_s
            point = {
                "stamp_s": stamp_s,
                "x_m": float(pose["x_m"]),
                "y_m": float(pose["y_m"]),
                "yaw_rad": float(pose["yaw_rad"]),
            }
            self.trajectory.append(point)
            del self.trajectory[:-2000]


HTML = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>Experimental Localization Monitor</title>
<style>
body{font:14px system-ui,sans-serif;margin:18px;background:#111827;color:#e5e7eb}
main{max-width:1120px;margin:auto}.grid{display:grid;grid-template-columns:repeat(6,1fr);gap:8px}
section{background:#1f2937;border-radius:10px;padding:12px;margin:10px 0}
.value{font-size:20px;font-weight:700}.muted{color:#9ca3af}.warn{color:#fbbf24}
canvas{width:100%;height:600px;background:#0b1220;border:1px solid #374151}
code{color:#bfdbfe}pre{white-space:pre-wrap}.state{letter-spacing:.08em}
</style></head><body><main>
<h1>Experimental candidate-map localization</h1>
<p class="muted">READ-ONLY GUI: subscribes/displays only. It publishes no TF,
initial pose, service request, or <code>/cmd_vel</code>.</p>
<section><div class="grid">
<div>State<div id="state" class="value state">WAIT</div></div>
<div>map X<div id="x" class="value">—</div></div>
<div>map Y<div id="y" class="value">—</div></div>
<div>Yaw<div id="yaw" class="value">—</div></div>
<div>XY covariance<div id="covxy" class="value">—</div></div>
<div>Yaw covariance<div id="covyaw" class="value">—</div></div>
</div></section>
<section><div class="grid">
<div>map→odom<div id="tf" class="value">WAIT</div></div>
<div>TF age<div id="tfage" class="value">—</div></div>
<div>Scan<div id="scanstate" class="value">WAIT</div></div>
<div>Scan points<div id="scancount" class="value">—</div></div>
<div>Map size<div id="mapsize" class="value">—</div></div>
<div>Map resolution<div id="mapres" class="value">—</div></div>
<div>重合狀態<div id="overlapstate" class="value">WAIT_MAP</div></div>
<div>≤0.15 m<div id="overlapnear" class="value">—</div></div>
<div>≤0.30 m<div id="overlapreview" class="value">—</div></div>
</div><p id="mapdetail" class="muted">Waiting for /map…</p></section>
<section><canvas id="scene" width="1120" height="600"></canvas>
<p class="muted">Blue: map-frame trajectory · red: AMCL pose/heading · green/yellow/red:
scan endpoints within 0.15 m / 0.30 m / unmatched of occupied map cells. A retained
scan is marked DELAY until a newer exact TF is available.</p></section>
<section><h2>State detail</h2><pre id="detail">Waiting for telemetry…</pre></section>
<section><h2>Manual initial-pose operation</h2>
<p>Use RViz or an operator terminal to publish <code>/initialpose</code> after the
map, <code>/scan</code>, and TF inputs are present. This GUI does not publish the
initial pose.</p>
<pre>ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \\
'{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'</pre>
<p class="warn">Replace the example pose with a measured pose. Map loading, a visible
pose/path, or an OK health state is not localization-accuracy acceptance.</p>
</section></main><script>
let mapRevision=-1,mapPixels=null;
const fmt=(v,d=3)=>v===null||v===undefined?'—':Number(v).toFixed(d);
function decode(s){const raw=atob(s),out=new Uint8Array(raw.length);for(let i=0;i<raw.length;i++)out[i]=raw.charCodeAt(i);return out;}
function localWorld(u,v,m){const c=Math.cos(m.origin.yaw_rad),s=Math.sin(m.origin.yaw_rad);return{x:m.origin.x_m+c*u-s*v,y:m.origin.y_m+s*u+c*v};}
function bounds(m){const p=[localWorld(0,0,m),localWorld(m.width*m.resolution_m,0,m),localWorld(0,m.height*m.resolution_m,m),localWorld(m.width*m.resolution_m,m.height*m.resolution_m,m)];return{minX:Math.min(...p.map(q=>q.x)),maxX:Math.max(...p.map(q=>q.x)),minY:Math.min(...p.map(q=>q.y)),maxY:Math.max(...p.map(q=>q.y))};}
function canvasPoint(q,b,c){const pad=34,dx=Math.max(b.maxX-b.minX,.1),dy=Math.max(b.maxY-b.minY,.1),scale=Math.min((c.width-2*pad)/dx,(c.height-2*pad)/dy);return{x:pad+(q.x-b.minX)*scale,y:c.height-pad-(q.y-b.minY)*scale,scale};}
function drawScene(d){const c=document.getElementById('scene'),x=c.getContext('2d');x.setTransform(1,0,0,1,0,0);x.clearRect(0,0,c.width,c.height);if(!d.map?.available||!mapPixels){x.fillStyle='#9ca3af';x.fillText('Waiting for /map occupancy grid…',24,30);return;}const b=bounds(d.map),pad=34,dx=Math.max(b.maxX-b.minX,.1),dy=Math.max(b.maxY-b.minY,.1),scale=Math.min((c.width-2*pad)/dx,(c.height-2*pad)/dy),originTop=canvasPoint(localWorld(0,d.map.height*d.map.resolution_m,d.map),b,c);let off=document.createElement('canvas');off.width=d.map.width;off.height=d.map.height;let oi=off.getContext('2d'),img=oi.createImageData(off.width,off.height);for(let v=0;v<d.map.height;v++)for(let u=0;u<d.map.width;u++){const gray=mapPixels[v*d.map.width+u],row=d.map.height-1-v,i=(row*d.map.width+u)*4;img.data[i]=gray;img.data[i+1]=gray;img.data[i+2]=gray;img.data[i+3]=255;}oi.putImageData(img,0,0);const ang=d.map.origin.yaw_rad,ca=Math.cos(ang),sa=Math.sin(ang);x.save();x.setTransform(ca*scale,-sa*scale,sa*scale,ca*scale,originTop.x,originTop.y);x.globalAlpha=.88;x.drawImage(off,0,0);x.restore();
 function line(points,color,width){if(!points?.length)return;x.strokeStyle=color;x.lineWidth=width;x.beginPath();points.forEach((p,i)=>{const q=canvasPoint(p,b,c);i?x.lineTo(q.x,q.y):x.moveTo(q.x,q.y)});x.stroke();}
 line(d.trajectory,'#60a5fa',2);if(d.scan_points?.length){d.scan_points.forEach(p=>{const q=canvasPoint(p,b,c);const color=p.overlap==='near'?'#22c55e':p.overlap==='review'?'#facc15':p.overlap==='unmatched'?'#ef4444':'#9ca3af';x.fillStyle=color;x.beginPath();x.arc(q.x,q.y,2,0,Math.PI*2);x.fill();});}
 if(d.pose){const q=canvasPoint({x:d.pose.x_m,y:d.pose.y_m},b,c),len=.32*q.scale;x.strokeStyle='#ef4444';x.fillStyle='#ef4444';x.lineWidth=3;x.beginPath();x.arc(q.x,q.y,5,0,Math.PI*2);x.fill();x.beginPath();x.moveTo(q.x,q.y);x.lineTo(q.x+Math.cos(d.pose.yaw_rad)*len,q.y-Math.sin(d.pose.yaw_rad)*len);x.stroke();const hx=q.x+Math.cos(d.pose.yaw_rad)*len,hy=q.y-Math.sin(d.pose.yaw_rad)*len;x.beginPath();x.moveTo(hx,hy);x.lineTo(hx-8*Math.cos(d.pose.yaw_rad-.55),hy+8*Math.sin(d.pose.yaw_rad-.55));x.lineTo(hx-8*Math.cos(d.pose.yaw_rad+.55),hy+8*Math.sin(d.pose.yaw_rad+.55));x.closePath();x.fill();}
 x.fillStyle='#d1d5db';x.fillText('map frame',12,20);}
async function refresh(){try{const t=await fetch('/api/telemetry',{cache:'no-store'}).then(r=>r.json()),d=await fetch('/api/render',{cache:'no-store'}).then(r=>r.json());if(d.map?.revision!==mapRevision&&d.map?.available){const g=await fetch('/api/map_grid?revision='+d.map.revision,{cache:'no-store'}).then(r=>r.json());mapPixels=decode(g.data);mapRevision=g.revision;}const o=t.map_overlap||t.scan.map_overlap||{};document.getElementById('state').textContent=t.state;document.getElementById('x').textContent=fmt(t.pose?.x_m);document.getElementById('y').textContent=fmt(t.pose?.y_m);document.getElementById('yaw').textContent=fmt(t.pose?.yaw_rad);document.getElementById('covxy').textContent=fmt(t.pose?.covariance?.xy_m2);document.getElementById('covyaw').textContent=fmt(t.pose?.covariance?.yaw_rad2);document.getElementById('tf').textContent=t.map_to_odom.available?'OK':'WAIT';document.getElementById('tfage').textContent=fmt(t.map_to_odom.age_s)+' s';document.getElementById('scanstate').textContent=t.scan.status;document.getElementById('scancount').textContent=fmt(t.scan.point_count,0)+' / '+fmt(t.scan.valid_input_count,0);document.getElementById('mapsize').textContent=t.map.available?`${t.map.width}×${t.map.height}`:'—';document.getElementById('mapres').textContent=t.map.available?fmt(t.map.resolution_m,3)+' m':'—';document.getElementById('overlapstate').textContent=o.status||'WAIT_MAP';document.getElementById('overlapnear').textContent=o.matched_near_ratio===null||o.matched_near_ratio===undefined?'—':(100*o.matched_near_ratio).toFixed(1)+'%';document.getElementById('overlapreview').textContent=o.matched_review_ratio===null||o.matched_review_ratio===undefined?'—':(100*o.matched_review_ratio).toFixed(1)+'%';document.getElementById('mapdetail').textContent=t.map.available?`origin [${fmt(t.map.origin.x_m)}, ${fmt(t.map.origin.y_m)}, yaw ${fmt(t.map.origin.yaw_rad)}] · map age ${fmt(t.map.age_s)} s · scan ${t.scan.status} ${t.scan.retained_last_valid?'(retained last exact sample)':''} · overlap ${o.status||'WAIT_MAP'}`:'Waiting for /map…';document.getElementById('detail').textContent=JSON.stringify({state:t.state,reasons:t.reasons,transition:t.state_transition,active_map_warning:t.active_map_warning,scan:t.scan,map_overlap:o},null,2);drawScene(d);}catch(e){document.getElementById('detail').textContent='Monitor unavailable: '+e;}}
refresh();setInterval(refresh,500);
</script></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802 - stdlib handler API
        monitor: MonitorSnapshot = self.server.monitor  # type: ignore[attr-defined]
        path = self.path.split("?", 1)[0]
        status = 200
        if path == "/api/telemetry":
            body = json.dumps(
                monitor.latest or {"schema_version": SCHEMA_VERSION},
                separators=(",", ":"),
            ).encode()
            content_type = "application/json"
        elif path == "/api/render":
            body = json.dumps(
                monitor.render_payload(monitor.latest.get("generated_at_s", 0.0)),
                separators=(",", ":"),
            ).encode()
            content_type = "application/json"
        elif path == "/api/map_grid":
            body = json.dumps(monitor.map_snapshot.grid_payload(), separators=(",", ":")).encode()
            content_type = "application/json"
        elif path == "/":
            body = HTML.encode()
            content_type = "text/html; charset=utf-8"
        else:
            status = 404
            body = b"not found"
            content_type = "text/plain"
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):
        return


class MonitorServer(ThreadingHTTPServer):
    allow_reuse_address = True

    def __init__(self, address, monitor):
        super().__init__(address, _Handler)
        self.monitor = monitor


def start_server(monitor: MonitorSnapshot, host: str, port: int) -> MonitorServer:
    server = MonitorServer((host, port), monitor)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def _pose_from_message(message: Any) -> dict[str, Any]:
    pose = message.pose.pose
    covariance = message.pose.covariance
    return {
        "frame_id": message.header.frame_id,
        "x_m": _json_number(pose.position.x),
        "y_m": _json_number(pose.position.y),
        "yaw_rad": _json_number(
            quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        ),
        "covariance": {
            "x_m2": _json_number(covariance[0]),
            "y_m2": _json_number(covariance[7]),
            "yaw_rad2": _json_number(covariance[35]),
            "xy_m2": _json_number(max(covariance[0], covariance[7])),
        },
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args, ros_args = parser.parse_known_args(argv)

    try:
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.time import Time
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError as exc:
        raise SystemExit(f"ROS 2 runtime is required for live monitoring: {exc}") from exc

    class NodeImpl(Node):
        def __init__(self):
            super().__init__("experimental_localization_monitor")
            self.snapshot = MonitorSnapshot()
            self.buffer = Buffer(cache_time=Duration(seconds=10.0))
            self.listener = TransformListener(self.buffer, self)
            self.create_subscription(
                PoseWithCovarianceStamped,
                "/amcl_pose",
                self.on_pose,
                10,
            )
            self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
            self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
            self.create_timer(0.2, self.on_timer)

        def _message_stamp(self, message):
            stamp_s = stamp_to_seconds(message.header.stamp)
            return stamp_s if stamp_s > 0.0 else self.get_clock().now().nanoseconds * 1e-9

        def on_pose(self, message):
            self.snapshot.accept_pose(_pose_from_message(message), self._message_stamp(message))

        def on_map(self, message):
            info = message.info
            stamp_s = self._message_stamp(message)
            self.snapshot.map_snapshot.accept_grid(
                width=info.width,
                height=info.height,
                resolution_m=info.resolution,
                origin_x_m=info.origin.position.x,
                origin_y_m=info.origin.position.y,
                origin_yaw_rad=quaternion_to_yaw(
                    info.origin.orientation.x,
                    info.origin.orientation.y,
                    info.origin.orientation.z,
                    info.origin.orientation.w,
                ),
                data=message.data,
                stamp_s=stamp_s,
                frame_id=message.header.frame_id or "map",
            )

        def on_scan(self, message):
            self.snapshot.scan_snapshot.accept_scan(
                stamp_s=self._message_stamp(message),
                stamp_msg=message.header.stamp,
                frame_id=message.header.frame_id,
                ranges=message.ranges,
                angle_min=message.angle_min,
                angle_increment=message.angle_increment,
                range_min=message.range_min,
                range_max=message.range_max,
            )

        def _lookup_scan_transform(self, entry):
            if not entry["frame_id"]:
                return None
            try:
                transform = self.buffer.lookup_transform(
                    "map",
                    entry["frame_id"],
                    Time.from_msg(entry["stamp_msg"]),
                )
            except TransformException:
                return None
            rotation = transform.transform.rotation
            return {
                "x_m": float(transform.transform.translation.x),
                "y_m": float(transform.transform.translation.y),
                "yaw_rad": quaternion_to_yaw(
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                ),
                "tf_stamp_s": stamp_to_seconds(transform.header.stamp),
            }

        def on_timer(self):
            now_s = self.get_clock().now().nanoseconds * 1e-9
            self.snapshot.scan_snapshot.resolve(self._lookup_scan_transform, now_s)
            tf_available = False
            tf_stamp_s = None
            try:
                transform = self.buffer.lookup_transform("map", "odom", Time())
                tf_available = True
                tf_stamp_s = stamp_to_seconds(transform.header.stamp)
            except TransformException:
                pass
            telemetry = self.snapshot.update(
                now_s=now_s,
                tf_available=tf_available,
                tf_stamp_s=tf_stamp_s,
            )
            print(
                "LOCALIZATION_TELEMETRY "
                + json.dumps(telemetry, sort_keys=True, separators=(",", ":")),
                flush=True,
            )

    rclpy.init(args=ros_args)
    node = NodeImpl()
    server = start_server(node.snapshot, args.host, args.port)
    node.get_logger().info(f"Read-only monitor available at http://{args.host}:{args.port}/")
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
