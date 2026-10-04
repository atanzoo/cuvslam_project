#!/usr/bin/env python3
"""Local browser GUI for the real D435i R1.5 axis test."""

from __future__ import annotations

import argparse
import errno
import json
import math
import os
import shlex
import shutil
import socket
import subprocess
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def load_local_env(path: Path) -> dict[str, str]:
    """Read optional local settings without adding a dotenv dependency."""
    if not path.exists():
        return {}
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].lstrip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
            value = value[1:-1]
        values[key] = value
    return values


LOCAL_ENV = load_local_env(PROJECT_ROOT / "shared" / "config" / "local.env")


def local_setting(name: str, default: str) -> str:
    return os.environ.get(name, LOCAL_ENV.get(name, default))


REMOTE_SCRIPT_CANDIDATES = (
    "/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_r1p5_axis.sh",
    "/home/tseng/isaac_ros_ws/tools/run_real_d435i_r1p5_axis.sh",
)
REMOTE_R2_SCRIPT_CANDIDATES = (
    "/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_r2.sh",
    "/home/tseng/isaac_ros_ws/tools/run_real_d435i_r2.sh",
)
REMOTE_RPLIDAR_SCRIPT_CANDIDATES = (
    "/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh",
    "/home/tseng/isaac_ros_ws/tools/run_real_d435i_rplidar.sh",
)
REMOTE_LOCALIZATION_SCRIPT_CANDIDATES = (
    "/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_experimental_localization.sh",
    "/home/tseng/isaac_ros_ws/tools/run_experimental_localization.sh",
)
REMOTE_RVIZ_VIEW_SCRIPT_CANDIDATES = (
    "/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/open_real_d435i_rviz_view.sh",
    "/home/tseng/isaac_ros_ws/tools/open_real_d435i_rviz_view.sh",
)
REMOTE_DATA_DIR = "/home/tseng/isaac_ros_data/real_robot/cuvslam/experiments/real_d435i_r1p5_gui"
REMOTE_R2_DATA_DIR = "/home/tseng/isaac_ros_data/real_robot/cuvslam/experiments/real_d435i_r2"
REMOTE_RPLIDAR_DATA_DIR = "/home/tseng/isaac_ros_data/real_robot/cuvslam/experiments/real_d435i_rplidar"
LOCAL_EVIDENCE_DIR = PROJECT_ROOT / "real_robot" / "cuvslam" / "evidence"
LOCAL_LOG_DIR = LOCAL_EVIDENCE_DIR / "logs" / "real_d435i_r1p5_gui"
QUALITY_LOG_DIR = LOCAL_EVIDENCE_DIR / "logs" / "real_d435i_quality"
LOCAL_R2_DATA_DIR = LOCAL_EVIDENCE_DIR / "logs" / "real_d435i_r2_downloads"
CORRECTION_LOG_PATH = QUALITY_LOG_DIR / "20260825_r2_correction_log.md"
HOST = local_setting("CUVSLAM_JETSON_HOST", "192.168.55.1")
USER = local_setting("CUVSLAM_JETSON_USER", "tseng")
VNC_PASSWORD = local_setting("CUVSLAM_VNC_PASSWORD", "")
# The physical A2M12 mounting is the current reference.  Keep this as a
# software TF override so the sensor does not need to be moved for calibration.
LIDAR_X = local_setting("CUVSLAM_LIDAR_X", "0.0")
LIDAR_Y = local_setting("CUVSLAM_LIDAR_Y", "0.0")
LIDAR_Z = local_setting("CUVSLAM_LIDAR_Z", "0.0")
LIDAR_YAW = local_setting("CUVSLAM_LIDAR_YAW", "0.0")

LIGHTING_REQUESTS: dict[str, dict[str, object]] = {
    "official": {
        "ir_projector": "ON", "auto_exposure": "ON",
        "exposure_us": 8500, "gain": 16, "denoise": "ON",
    },
    "low_light": {
        "ir_projector": "ON", "auto_exposure": "ON",
        "exposure_us": 8500, "gain": 16, "denoise": "ON",
    },
    "low_light_motion": {
        "ir_projector": "ON", "auto_exposure": "OFF",
        "exposure_us": 10000, "gain": 24, "denoise": "ON",
    },
    "baseline": {
        "ir_projector": "ON", "auto_exposure": "ON",
        "exposure_us": 8500, "gain": 16, "denoise": "OFF",
    },
    "robust": {
        "ir_projector": "OFF", "auto_exposure": "ON",
        "exposure_us": 8500, "gain": 16, "denoise": "ON",
    },
    "high_contrast": {
        "ir_projector": "OFF", "auto_exposure": "OFF",
        "exposure_us": 8000, "gain": 16, "denoise": "ON",
    },
}


GEOMETRY_GATE_RANK = {
    "未偵測": 0,
    "PENDING": 0,
    "PASS": 1,
    "REVIEW": 2,
    "FAIL": 3,
}


def requested_runtime_settings(
    mode: str, mapping: str, profile: str, lighting: str, diagnostics: str,
    scene: str, motion: str, estimator_stack: str = "cuvslam",
    emitter_override: str = "profile",
) -> dict[str, object]:
    lighting_values = dict(LIGHTING_REQUESTS.get(lighting, {}))
    if emitter_override in {"on", "off"}:
        lighting_values["ir_projector"] = "ON" if emitter_override == "on" else "OFF"
    lighting_values["emitter_override"] = emitter_override
    return {
        "mode": mode,
        "mapping": mapping,
        "profile": profile,
        "lighting": lighting,
        "diagnostics": diagnostics,
        "scene_label": scene,
        "motion_label": motion,
        "estimator_stack": estimator_stack,
        "lidar_mount_requested": {
            "parent_frame": "base_link",
            "child_frame": "rplidar_link",
            "x_m": LIDAR_X,
            "y_m": LIDAR_Y,
            "z_m": LIDAR_Z,
            "yaw_rad": LIDAR_YAW,
            "physical_mount": "unchanged",
            "calibration_status": "software_override_not_accuracy_qualified",
        },
        "d435i_requested": lighting_values,
        "cuvslam_requested": {
            "observations_view": diagnostics == "features",
            "landmarks_pose_graph_topics": "full recording only",
        },
    }
PHASES = {
    "optical_x_up": ("X 軸：鏡頭向上抬", "把鏡頭向上抬 60–90°，約 1–2 秒完成"),
    "optical_x_down": ("X 軸反向：鏡頭向下壓", "把鏡頭向下壓回原位，約 1–2 秒完成"),
    "optical_y_left": ("Y 軸：鏡頭向左轉", "像搖頭一樣向左轉 60–90°，約 1–2 秒完成"),
    "optical_y_right": ("Y 軸反向：鏡頭向右轉", "向右轉回原位，約 1–2 秒完成"),
    "optical_z_clockwise": ("Z 軸：畫面順時針扭轉", "在畫面平面內順時針扭轉 60–90°"),
    "optical_z_counterclockwise": ("Z 軸反向：畫面逆時針扭轉", "逆時針扭轉回原位"),
}


HTML = r"""<!doctype html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Real D435i · R1.5 三軸 IMU 測試</title>
<style>
:root { color-scheme: dark; font-family: -apple-system, BlinkMacSystemFont, "Helvetica Neue", sans-serif; }
* { box-sizing: border-box; }
body { margin: 0; background: #0f172a; color: #e2e8f0; }
.page { max-width: 1440px; margin: 0 auto; padding: 24px; }
h1 { margin: 0 0 18px; font-size: 26px; color: #f8fafc; }
h2 { margin: 0 0 12px; font-size: 16px; color: #f8fafc; }
.panel { background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 16px; box-shadow: 0 8px 24px #02061755; }
.connection { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; margin-bottom: 16px; }
.connection span { color: #cbd5e1; }
.section-heading { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 12px; }
.section-heading h2 { margin: 0; }
.section-toggle { padding: 6px 10px; font-size: 12px; background: #334155; color: #cbd5e1; }
.section-toggle:hover:not(:disabled) { background: #475569; }
.collapsible-body[hidden] { display: none; }
input { width: 72px; padding: 9px 10px; border: 1px solid #475569; border-radius: 7px; background: #f8fafc; color: #0f172a; }
select { padding: 9px 10px; border: 1px solid #475569; border-radius: 7px; background: #f8fafc; color: #0f172a; }
button { border: 0; border-radius: 7px; padding: 10px 14px; color: #f8fafc; background: #475569; cursor: pointer; font-size: 14px; }
button:hover:not(:disabled) { background: #64748b; }
button:disabled { opacity: .4; cursor: not-allowed; }
.primary { background: #2563eb; }
.primary:hover:not(:disabled) { background: #1d4ed8; }
.danger { background: #be123c; }
.status { color: #93c5fd; min-height: 24px; margin: 10px 0 16px; }
.grid { display: grid; grid-template-columns: 300px minmax(380px, 1fr) 360px; gap: 16px; align-items: stretch; }
.phase-list { display: grid; gap: 8px; }
.phase-list button { text-align: left; }
.phase-help { min-height: 64px; padding: 10px; margin-bottom: 10px; border-radius: 8px; background: #0f172a; color: #cbd5e1; line-height: 1.5; }
.metric { font: 15px ui-monospace, SFMono-Regular, Menlo, monospace; margin: 7px 0; }
.bars { display: grid; gap: 16px; margin-top: 22px; }
.bar-row { display: grid; grid-template-columns: 32px 1fr 72px; gap: 8px; align-items: center; font: 14px ui-monospace, monospace; }
.track { height: 18px; border-radius: 5px; background: #0f172a; overflow: hidden; position: relative; }
.zero { position: absolute; left: 50%; top: 0; bottom: 0; width: 1px; background: #64748b; }
.bar { position: absolute; top: 2px; height: 14px; border-radius: 4px; background: #38bdf8; }
.bar.negative { background: #fb7185; }
pre { margin: 0; min-height: 110px; max-height: 250px; overflow: auto; padding: 10px; border-radius: 8px; background: #0f172a; color: #cbd5e1; white-space: pre-wrap; word-break: break-word; font: 12px ui-monospace, monospace; }
.results { margin-top: 16px; }
.r2-panel { margin-bottom: 16px; }
.r2-controls { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }
.mapping-record-controls { margin-top: 10px; padding-top: 10px; border-top: 1px solid #334155; }
.r2-status { color: #93c5fd; min-height: 22px; margin: 12px 0; }
.r2-grid { display: grid; grid-template-columns: 340px 1fr; gap: 16px; align-items: stretch; }
.odom-metric { display: grid; grid-template-columns: 100px 1fr; gap: 8px; font: 15px ui-monospace, SFMono-Regular, Menlo, monospace; margin: 10px 0; }
.odom-metric span:first-child { color: #94a3b8; }
.odom-canvas { width: 100%; height: 300px; display: block; border-radius: 8px; background: #0f172a; border: 1px solid #334155; }
.mapping-workspace { margin-top: 16px; padding: 12px; border-radius: 9px; background: #0f172a; border: 1px solid #334155; }
.mapping-metrics { display: grid; grid-template-columns: repeat(4, minmax(150px, 1fr)); gap: 10px; margin-bottom: 12px; }
.mapping-metric { padding: 9px 10px; border-radius: 7px; background: #1e293b; font: 13px ui-monospace, SFMono-Regular, Menlo, monospace; }
.mapping-metric span { display: block; color: #94a3b8; margin-bottom: 4px; }
.mapping-metric strong { color: #e2e8f0; font-size: 14px; }
.mapping-canvas { width: 100%; height: 460px; display: block; border-radius: 8px; background: #0b1120; border: 1px solid #334155; }
.localization-panel { margin: 16px 0; }
.localization-controls { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; margin-bottom: 12px; }
.localization-controls strong { color: #f8fafc; }
.localization-canvas { width: 100%; height: 520px; display: block; border-radius: 8px; background: #0b1120; border: 1px solid #334155; }
.r2-note { color: #94a3b8; font-size: 12px; margin-top: 10px; }
.motion-workspace { margin: 16px 0; padding: 12px; border-radius: 9px; background: #0f172a; border: 1px solid #334155; }
.motion-controls { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
.motion-controls strong { color: #f8fafc; }
.motion-metrics { display: grid; grid-template-columns: repeat(5, minmax(130px, 1fr)); gap: 10px; margin-top: 12px; }
.motion-metric { padding: 9px 10px; border-radius: 7px; background: #1e293b; font: 13px ui-monospace, SFMono-Regular, Menlo, monospace; }
.motion-metric span { display: block; color: #94a3b8; margin-bottom: 4px; }
.motion-metric strong { color: #e2e8f0; font-size: 15px; }
.quality-workspace { margin: 16px 0 0; padding: 12px; border-radius: 9px; background: #0f172a; border: 1px solid #334155; }
.quality-header { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }
.quality-header strong { color: #f8fafc; }
.quality-metrics { display: grid; grid-template-columns: repeat(6, minmax(120px, 1fr)); gap: 10px; margin-top: 12px; }
.quality-metric { padding: 9px 10px; border-radius: 7px; background: #1e293b; font: 12px ui-monospace, SFMono-Regular, Menlo, monospace; }
.quality-metric span { display: block; color: #94a3b8; margin-bottom: 4px; }
.quality-metric strong { color: #e2e8f0; font-size: 14px; }
.assessment-workspace { margin: 16px 0 0; padding: 12px; border-radius: 9px; background: #0f172a; border: 1px solid #334155; }
.assessment-header { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }
.assessment-header strong { color: #f8fafc; }
.assessment-score { font: 700 20px ui-monospace, SFMono-Regular, Menlo, monospace; color: #e2e8f0; }
.assessment-standards { display: grid; grid-template-columns: repeat(4, minmax(180px, 1fr)); gap: 10px; margin-top: 10px; }
.assessment-standard { padding: 9px 10px; border-radius: 7px; background: #1e293b; font: 12px ui-monospace, SFMono-Regular, Menlo, monospace; }
.assessment-standard span { display: block; color: #94a3b8; margin-bottom: 4px; }
.assessment-standard strong { color: #e2e8f0; font-size: 13px; }
.log-actions { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; margin-bottom: 10px; }
.copy-status { color: #86efac; font-size: 12px; }
.mapping-summary { min-height: 260px; max-height: 420px; }
.process-log { min-height: 220px; max-height: 460px; }
.log-summary-heading { margin-top: 14px; margin-bottom: 8px; }
.log-summary-heading .log-actions { margin-bottom: 0; }
.quality-good { color: #86efac !important; }
.quality-warn { color: #fbbf24 !important; }
.quality-bad { color: #fb7185 !important; }
@media (max-width: 1050px) { .grid { grid-template-columns: 1fr 1fr; } .log-panel { grid-column: 1 / -1; } }
@media (max-width: 700px) { .grid { grid-template-columns: 1fr; } .log-panel { grid-column: auto; } .motion-metrics, .quality-metrics, .assessment-standards { grid-template-columns: 1fr 1fr; } .page { padding: 12px; } }
</style>
</head>
<body>
<main class="page">
  <h1>Real D435i · R1.5 / R2 實機測試</h1>
  <section class="panel connection-panel">
    <div class="section-heading">
      <h2>連線與基本控制</h2>
      <button class="section-toggle" type="button" aria-controls="connection-body" aria-expanded="false" onclick="toggleSection('connection-body', this)">展開</button>
    </div>
    <div id="connection-body" class="collapsible-body" hidden>
      <div class="connection">
        <span>Jetson：tseng@192.168.55.1（USB）</span>
        <span>每段秒數</span><input id="duration" type="number" min="3" max="60" value="10">
        <button id="start" class="primary" onclick="startRig()">啟動實機測試</button>
        <button class="danger" onclick="stopRig()">停止</button>
      </div>
    </div>
  </section>
  <div id="status" class="status">尚未啟動測試容器</div>
  <section class="panel localization-panel" aria-labelledby="localization-heading">
    <div class="section-heading">
      <h2 id="localization-heading">定位測試區</h2>
      <span id="localization-status" class="r2-note">定位尚未啟動</span>
    </div>
    <div class="localization-controls">
      <button class="primary" id="localization-start" onclick="startLocalization()">開始定位</button>
      <button class="danger" id="localization-stop" onclick="stopLocalization()">停止定位</button>
      <strong>只啟動實驗定位節點</strong>
      <span class="r2-note">需要外部 /scan 與 odom；本區不控制馬達，也不發布 /cmd_vel。</span>
    </div>
    <div class="mapping-metrics">
      <div class="mapping-metric"><span>地圖資訊</span><strong id="localization-map-size">—</strong></div>
      <div class="mapping-metric"><span>地圖 frame / resolution</span><strong id="localization-map-frame">—</strong></div>
      <div class="mapping-metric"><span>AMCL 定位</span><strong id="localization-pose-status">WAIT_AMCL</strong></div>
      <div class="mapping-metric"><span>定位座標</span><strong id="localization-pose">—</strong></div>
      <div class="mapping-metric"><span>scan-map 重合</span><strong id="localization-alignment">WAIT</strong></div>
      <div class="mapping-metric"><span>scan / TF</span><strong id="localization-scan-status">—</strong></div>
    </div>
    <canvas id="localization-canvas" class="localization-canvas" width="1100" height="520" role="img" aria-label="定位點、雷射掃描與實際 occupancy map 重合檢視"></canvas>
    <div class="r2-note">底圖：/map occupancy grid；紅色箭頭：AMCL 定位；綠色／黃色／紅色：雷射端點距離已佔用地圖 ≤0.15 m／≤0.30 m／未貼合。沒有完整資料時維持 WAIT，不把畫面當成定位精度驗收。</div>
  </section>
  <section class="panel r2-panel">
    <div class="section-heading">
      <h2>R2 / 2D SLAM 即時觀察區</h2>
      <button class="section-toggle" type="button" aria-controls="r2-controls-body" aria-expanded="false" onclick="toggleSection('r2-controls-body', this)">展開設定</button>
    </div>
    <div id="r2-status" class="r2-status">R2 尚未啟動</div>
    <div id="r2-controls-body" class="collapsible-body" hidden>
      <div class="r2-controls">
        <span>模式</span>
        <select id="r2-mode">
          <option value="stereo">Stereo-only</option>
          <option value="imu" selected>Stereo + IMU fusion（正式建圖預設）</option>
        </select>
        <span>工作</span>
        <select id="r2-work">
          <option value="odom">odom-only（距離校正）</option>
          <option value="mapping" selected>mapping（建圖）</option>
        </select>
        <span>影像</span>
        <select id="r2-profile">
          <option value="640x360x30">640×360×30（實機推薦）</option>
          <option value="848x480x30">848×480×30</option>
        </select>
        <span>光照</span>
        <select id="r2-lighting">
          <option value="official" selected>NVIDIA official low-light：去噪 + auto exposure</option>
          <option value="robust">診斷：去噪 + 關 IR projector</option>
          <option value="low_light">Low-light baseline：去噪 + 開 IR projector</option>
          <option value="low_light_motion">Low-light motion（實驗：10 ms + gain 24）</option>
          <option value="high_contrast">診斷：手動曝光 8000（不建議正式建圖）</option>
          <option value="baseline">Baseline：原始設定</option>
        </select>
        <span>IR projector A/B</span>
        <select id="r2-emitter">
          <option value="on" selected>A：ON（開啟投射器）</option>
          <option value="off">B：OFF（關閉投射器）</option>
        </select>
        <span>場景標籤</span><input id="r2-scene-label" type="text" value="scene_unknown" maxlength="48" placeholder="例如 special_box_scene">
        <span>動作標籤</span>
        <select id="r2-motion-label">
          <option value="forward_left_turn">前進後左轉</option>
          <option value="forward_right_turn">前進後右轉</option>
          <option value="pure_turn">原地旋轉</option>
          <option value="straight">直線平移</option>
          <option value="closed_loop">閉環路徑</option>
          <option value="other" selected>其他</option>
        </select>
        <span>特徵診斷</span>
        <select id="r2-diagnostics">
          <option value="standard" selected>關閉（正式建圖、負載較低）</option>
          <option value="features">開啟（特殊場景，輸出 observations）</option>
        </select>
        <span>估計器</span>
        <select id="r2-estimator">
          <option value="cuvslam" selected>cuVSLAM（目前正式 odom）</option>
          <option value="ekf_shadow">LiDAR + EKF shadow（診斷，不接管 TF）</option>
        </select>
        <button class="primary" id="r2-start" onclick="startR2()">啟動目前工作</button>
        <button class="danger" id="r2-stop" onclick="stopR2()">停止目前工作並清除</button>
        <button class="primary" id="rviz-open" onclick="openRviz()">開啟可操控 RViz</button>
      </div>
      <div id="rviz-status" class="r2-note">RViz 尚未開啟；按鈕會自動開啟可操控的 macOS 螢幕共享</div>
      <div class="r2-controls mapping-record-controls">
        <span>錄製</span>
        <select id="r2-record-mode">
          <option value="light" selected>輕量建圖（推薦，不含原始 IR 影像）</option>
          <option value="full">完整回放（含左右原始 IR，檔案很大）</option>
        </select>
        <button class="primary" id="r2-record-start" onclick="startR2Recording()">開始錄製建圖資料</button>
        <button id="r2-record-stop" onclick="stopR2Recording()">停止錄製</button>
        <button id="r2-download" onclick="downloadR2Recording()">下載本次建圖資料</button>
        <span id="r2-record-status" class="r2-note">尚未錄製建圖資料</span>
      </div>
      <div id="r2-download-status" class="r2-note">輕量模式下載：CameraInfo、IMU、TF、odom、VO pose、SLAM path、landmarks、pose graph、loop closure 與 GUI 摘要；啟動「特徵診斷」才會有 observations；完整模式才包含原始 IR 影像。</div>
    </div>
    <div class="motion-workspace">
      <div class="section-heading">
        <strong>移動量記錄工作區</strong>
        <button class="section-toggle" type="button" aria-controls="motion-body" aria-expanded="false" onclick="toggleSection('motion-body', this)">展開</button>
      </div>
      <div id="motion-body" class="collapsible-body" hidden>
        <div class="motion-controls">
          <button class="primary" id="motion-start" onclick="startMotionRecord()">開始記錄</button>
          <button id="motion-stop" onclick="stopMotionRecord()">停止並保留</button>
          <button class="danger" id="motion-clear" onclick="clearMotionRecord()">清除</button>
          <span id="motion-status" class="r2-note">尚未記錄</span>
        </div>
        <div class="motion-metrics">
          <div class="motion-metric"><span>記錄時間</span><strong id="motion-duration">0.0 s</strong></div>
          <div class="motion-metric"><span>起點→終點</span><strong id="motion-displacement">0.000 m</strong></div>
          <div class="motion-metric"><span>累積路徑</span><strong id="motion-path-length">0.000 m</strong></div>
          <div class="motion-metric"><span>最大位移</span><strong id="motion-max">0.000 m</strong></div>
          <div class="motion-metric"><span>整段 yaw Δ</span><strong id="motion-yaw-delta">0.0°</strong></div>
          <div class="motion-metric"><span>samples / gap</span><strong id="motion-samples">0 / 0.0 ms</strong></div>
        </div>
        <div class="r2-note">本次記錄會保存整段 XY 路徑、整段 yaw、轉向片段、影像品質快照與事件時間戳。按「停止目前工作並清除」只清除即時工作區，已保存的 JSONL／session 檔案不會刪除。</div>
      </div>
    </div>
    <div class="quality-workspace">
      <div class="section-heading">
        <strong>影像品質（光照診斷）</strong>
        <div>
          <span id="quality-flag" class="r2-note">尚未連線</span>
          <button class="section-toggle" type="button" aria-controls="quality-body" aria-expanded="false" onclick="toggleSection('quality-body', this)">展開</button>
        </div>
      </div>
      <div id="quality-body" class="collapsible-body" hidden>
        <div class="quality-metrics">
          <div class="quality-metric"><span>左 mean / std</span><strong id="quality-left-level">—</strong></div>
          <div class="quality-metric"><span>右 mean / std</span><strong id="quality-right-level">—</strong></div>
          <div class="quality-metric"><span>暗部 / 飽和</span><strong id="quality-clipping">—</strong></div>
          <div class="quality-metric"><span>紋理代理 gradient</span><strong id="quality-gradient">—</strong></div>
          <div class="quality-metric"><span>影像 rate L / R</span><strong id="quality-rate">—</strong></div>
          <div class="quality-metric"><span>影像 gap P95</span><strong id="quality-gap">—</strong></div>
          <div class="quality-metric"><span>左右同步 P95</span><strong id="quality-stereo-sync">—</strong></div>
          <div class="quality-metric"><span>實際曝光 L / R</span><strong id="quality-exposure">—</strong></div>
          <div class="quality-metric"><span>實際 gain L / R</span><strong id="quality-gain">—</strong></div>
        </div>
        <div class="r2-note">mean 太低/暗部過高＝低光；飽和過高＝過曝；gradient 太低＝場景缺少可追蹤紋理；左右同步 P95 建議 ≤100 µs。這些數值只做診斷，不會改寫 IMU 或 odom。</div>
      </div>
    </div>
    <div class="assessment-workspace">
      <div class="section-heading">
        <strong>R2 數值品質判定</strong>
        <div>
          <span id="assessment-score" class="assessment-score">— / 100</span>
          <span id="assessment-status" class="r2-note">尚未連線</span>
          <button class="section-toggle" type="button" aria-controls="assessment-body" aria-expanded="false" onclick="toggleSection('assessment-body', this)">展開</button>
        </div>
      </div>
      <div id="assessment-body" class="collapsible-body" hidden>
        <div class="quality-metrics">
          <div class="quality-metric"><span>VO 有效率</span><strong id="assessment-vo">—</strong></div>
          <div class="quality-metric"><span>odom rate</span><strong id="assessment-rate">—</strong></div>
          <div class="quality-metric"><span>影像 gap P95</span><strong id="assessment-gap">—</strong></div>
          <div class="quality-metric"><span>左右同步 P95</span><strong id="assessment-stereo-sync">—</strong></div>
          <div class="quality-metric"><span>最大 clipped</span><strong id="assessment-clipping">—</strong></div>
          <div class="quality-metric"><span>完整路徑點</span><strong id="assessment-path">—</strong></div>
          <div class="quality-metric"><span>轉向抖動診斷</span><strong id="assessment-jitter">—</strong></div>
          <div class="quality-metric"><span>轉向平移診斷</span><strong id="assessment-turn-translation">—</strong></div>
          <div class="quality-metric"><span>LiDAR odom / EKF shadow</span><strong id="assessment-estimator">—</strong></div>
          <div class="quality-metric"><span>估計器 ΔXY / Δyaw</span><strong id="assessment-estimator-delta">—</strong></div>
          <div class="quality-metric"><span>EKF 內部 processed / rejected</span><strong id="assessment-ekf-internal">—</strong></div>
          <div class="quality-metric"><span>目前警告</span><strong id="assessment-warning">—</strong></div>
        </div>
        <div class="assessment-standards">
          <div class="assessment-standard"><span>60 cm 直線距離誤差</span><strong>Pass ≤10% · Review ≤15%</strong></div>
          <div class="assessment-standard"><span>90° 轉彎 yaw 誤差</span><strong>Pass ≤5° · Review ≤10°</strong></div>
          <div class="assessment-standard"><span>閉環終點位置誤差</span><strong>Pass ≤10 cm · Review ≤20 cm</strong></div>
          <div class="assessment-standard"><span>閉環 yaw 誤差</span><strong>Pass ≤5° · Review ≤10°</strong></div>
        </div>
        <div id="assessment-confidence" class="r2-note">VO 統計來源：—</div>
        <div class="r2-note">總分是即時影像與追蹤穩定度的工程判定，不代表幾何精度已通過；距離、轉彎與閉環要在對應測試完成後，以移動量記錄和完整路徑計算。轉向平移先顯示觀測值與「原地旋轉候選」；若該段實際包含平移，不能套用原地旋轉門檻。</div>
      </div>
    </div>
    <div class="r2-grid">
      <div>
        <div class="odom-metric"><span>x</span><strong id="odom-x">+0.000 m</strong></div>
        <div class="odom-metric"><span>y</span><strong id="odom-y">+0.000 m</strong></div>
        <div class="odom-metric"><span>z</span><strong id="odom-z">+0.000 m</strong></div>
        <div class="odom-metric"><span>yaw</span><strong id="odom-yaw">+0.00°</strong></div>
        <div class="odom-metric"><span>odom rate</span><strong id="odom-rate">0.0 Hz</strong></div>
        <div class="odom-metric"><span>telemetry</span><strong id="odom-telemetry">未連線</strong></div>
        <div class="odom-metric"><span>vo_state</span><strong id="odom-state">—</strong></div>
        <div class="r2-note">軌跡圖為共用 odom frame 的即時 XY 投影：cuVSLAM、LiDAR odom、EKF shadow。EKF shadow 僅供比較，不發布 TF，也不接管正式 odom。</div>
      </div>
      <canvas id="odom-canvas" class="odom-canvas" width="820" height="300"></canvas>
    </div>
    <div class="mapping-workspace">
      <div class="section-heading">
        <strong>2D LiDAR 建圖（即時觀察）</strong>
        <span id="mapping-status" class="r2-note">尚未收到 /map</span>
      </div>
      <div class="mapping-metrics">
        <div class="mapping-metric"><span>地圖</span><strong id="mapping-map-size">—</strong></div>
        <div class="mapping-metric"><span>地圖 frame</span><strong id="mapping-map-frame">—</strong></div>
        <div class="mapping-metric"><span>LiDAR scan</span><strong id="mapping-scan-status">—</strong></div>
        <div class="mapping-metric"><span>scan-map 貼合</span><strong id="mapping-alignment-status">—</strong></div>
        <div class="mapping-metric"><span>map frame 路徑點</span><strong id="mapping-path-points">0</strong></div>
      </div>
      <canvas id="mapping-canvas" class="mapping-canvas" width="1100" height="460"></canvas>
      <div class="r2-note">圖層：slam_toolbox `/map` occupancy grid、以完整 timestamped `map → rplidar_link` TF 延遲顯示的 `/scan`（最多 720 點）、以及以各筆 timestamped `map → odom` 投影的 cuVSLAM 路徑。scan-map 貼合顯示 0.15／0.30 m endpoint 比例與方向候選。選擇「LiDAR + EKF shadow」時另行比較 `/lidar/odom` 與 `/odometry/filtered_shadow`；shadow 不發布 TF，也不接管正式 odom。此圖是即時觀察，不是外參或地圖精度驗收。</div>
    </div>
  </section>
  <section class="grid">
    <section class="panel">
      <div class="section-heading">
        <h2>即時 IMU</h2>
        <button class="section-toggle" type="button" aria-controls="imu-body" aria-expanded="false" onclick="toggleSection('imu-body', this)">展開</button>
      </div>
      <div id="imu-body" class="collapsible-body" hidden>
        <div id="live" class="metric">gx +0.000 · gy +0.000 · gz +0.000 rad/s</div>
        <div id="peak" class="metric">peak 0.000 · dominant axis —</div>
        <div id="elapsed" class="metric">elapsed 0.0 s</div>
        <div class="bars">
          <div class="bar-row"><span>gx</span><div class="track"><i class="zero"></i><i id="bar0" class="bar"></i></div><span id="value0">+0.000</span></div>
          <div class="bar-row"><span>gy</span><div class="track"><i class="zero"></i><i id="bar1" class="bar"></i></div><span id="value1">+0.000</span></div>
          <div class="bar-row"><span>gz</span><div class="track"><i class="zero"></i><i id="bar2" class="bar"></i></div><span id="value2">+0.000</span></div>
        </div>
      </div>
    </section>
    <section class="panel log-panel">
      <div class="section-heading">
        <h2>工作記錄（過程）</h2>
        <button class="section-toggle" type="button" aria-controls="logs-body" aria-expanded="false" onclick="toggleSection('logs-body', this)">展開</button>
      </div>
      <div id="logs-body" class="collapsible-body" hidden>
        <div class="r2-note">依時間排列的啟動、連線、測試、警告與清理事件；這裡保留過程，不作結論摘要。</div>
        <div class="log-actions">
          <button type="button" onclick="copyProcessLog()">複製工作記錄（過程）</button>
          <button type="button" onclick="clearProcessLog()">清除工作記錄（過程）</button>
          <span id="process-copy-status" class="copy-status"></span>
          <span id="process-clear-status" class="copy-status"></span>
        </div>
        <pre id="logs" class="process-log"></pre>
        <div class="section-heading log-summary-heading">
          <strong>建圖分析摘要（結論）</strong>
          <div class="log-actions">
            <button class="primary" type="button" onclick="copyMappingSummary()">複製建圖分析摘要</button>
            <span id="copy-status" class="copy-status"></span>
          </div>
        </div>
        <pre id="mapping-summary" class="mapping-summary">尚未有 R2 建圖資料</pre>
      </div>
    </section>
  </section>
  <section class="panel results">
    <div class="section-heading">
      <h2>測試結果</h2>
      <button class="section-toggle" type="button" aria-controls="results-body" aria-expanded="false" onclick="toggleSection('results-body', this)">展開</button>
    </div>
    <div id="results-body" class="collapsible-body" hidden><pre id="results"></pre></div>
  </section>
</main>
<script>
const phaseHelp = {
  optical_x_up: ["X 軸：鏡頭向上抬", "把鏡頭向上抬 60–90°，約 1–2 秒完成"],
  optical_x_down: ["X 軸反向：鏡頭向下壓", "把鏡頭向下壓回原位，約 1–2 秒完成"],
  optical_y_left: ["Y 軸：鏡頭向左轉", "像搖頭一樣向左轉 60–90°，約 1–2 秒完成"],
  optical_y_right: ["Y 軸反向：鏡頭向右轉", "向右轉回原位，約 1–2 秒完成"],
  optical_z_clockwise: ["Z 軸：畫面順時針扭轉", "在畫面平面內順時針扭轉 60–90°"],
  optical_z_counterclockwise: ["Z 軸反向：畫面逆時針扭轉", "逆時針扭轉回原位"]
};
function setStatus(s) { document.getElementById('status').textContent = s; }
function post(path, body) { return fetch(path, {method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(body || {})}).then(r => r.json()); }
async function copyMappingSummary() {
  const text = document.getElementById('mapping-summary').textContent || '';
  const status = document.getElementById('copy-status');
  await copyToClipboard(text, status);
}
async function copyProcessLog() {
  const text = document.getElementById('logs').textContent || '';
  const status = document.getElementById('process-copy-status');
  await copyToClipboard(`=== 工作記錄（過程） ===\n${text}`, status);
}
function clearProcessLog() {
  post('/api/logs/clear').then(x => {
    document.getElementById('process-clear-status').textContent = x.message || '已清除';
    setTimeout(() => { document.getElementById('process-clear-status').textContent = ''; }, 1800);
  }).catch(() => {
    document.getElementById('process-clear-status').textContent = '清除失敗';
  });
}
async function copyToClipboard(text, status) {
  try {
    await navigator.clipboard.writeText(text);
  } catch (_) {
    const area = document.createElement('textarea');
    area.value = text; document.body.appendChild(area); area.select();
    document.execCommand('copy'); area.remove();
  }
  status.textContent = '已複製';
  setTimeout(() => { status.textContent = ''; }, 1800);
}
function toggleSection(id, button) {
  const body = document.getElementById(id);
  const expanded = body.hidden;
  body.hidden = !expanded;
  button.textContent = expanded ? '收合' : '展開';
  button.setAttribute('aria-expanded', expanded ? 'true' : 'false');
  try { localStorage.setItem(`cuvslam-section-${id}`, expanded ? 'open' : 'closed'); } catch (_) {}
}
function restoreSections() {
  document.querySelectorAll('.collapsible-body').forEach(body => {
    let saved = null;
    try { saved = localStorage.getItem(`cuvslam-section-${body.id}`); } catch (_) {}
    const button = document.querySelector(`[aria-controls="${body.id}"]`);
    if (!button) return;
    const open = saved === 'open';
    body.hidden = !open;
    button.textContent = open ? '收合' : '展開';
    button.setAttribute('aria-expanded', open ? 'true' : 'false');
  });
}
function startR2() {
  const mode = document.getElementById('r2-mode').value;
  const mapping = document.getElementById('r2-work').value;
  const profile = document.getElementById('r2-profile').value;
  const lighting = document.getElementById('r2-lighting').value;
  const emitter = document.getElementById('r2-emitter').value;
  const scene = document.getElementById('r2-scene-label').value.trim() || 'scene_unknown';
  const motion = document.getElementById('r2-motion-label').value;
  const diagnostics = document.getElementById('r2-diagnostics').value;
  const estimator = document.getElementById('r2-estimator').value;
  post('/api/r2/start', {mode, mapping, profile, lighting, emitter, scene, motion, diagnostics, estimator}).then(x => document.getElementById('r2-status').textContent = x.message || 'R2 啟動中');
}
function startLocalization() {
  post('/api/localization/start').then(x => document.getElementById('localization-status').textContent = x.message || '定位啟動中');
}
function stopLocalization() {
  post('/api/localization/stop').then(x => document.getElementById('localization-status').textContent = x.message || '定位停止中');
}
function stopR2() {
  post('/api/r2/stop').then(x => document.getElementById('r2-status').textContent = x.message || 'R2 停止中');
}
function startR2Recording() {
  const mode = document.getElementById('r2-record-mode').value;
  post('/api/r2/record/start', {mode}).then(x => document.getElementById('r2-record-status').textContent = x.message || '開始錄製中');
}
function stopR2Recording() {
  post('/api/r2/record/stop').then(x => document.getElementById('r2-record-status').textContent = x.message || '停止錄製中');
}
function downloadR2Recording() {
  post('/api/r2/record/download').then(x => document.getElementById('r2-download-status').textContent = x.message || '下載中');
}
function openRviz() {
  document.getElementById('rviz-status').textContent = '正在開啟 RViz 與可操控視窗…';
  post('/api/rviz/open').then(x => document.getElementById('rviz-status').textContent = x.message || 'RViz 開啟中');
}
function startMotionRecord() {
  post('/api/r2/motion/start').then(x => document.getElementById('motion-status').textContent = x.message || '開始記錄');
}
function stopMotionRecord() {
  post('/api/r2/motion/stop').then(x => document.getElementById('motion-status').textContent = x.message || '已停止並保留');
}
function clearMotionRecord() {
  post('/api/r2/motion/clear').then(x => document.getElementById('motion-status').textContent = x.message || '已清除');
}
function startRig() { post('/api/start').then(x => setStatus(x.message || '啟動中')); }
function stopRig() { post('/api/stop').then(x => setStatus(x.message || '停止中')); }
function runPhase(button) {
  const phase = button.dataset.phase;
  const duration = Number(document.getElementById('duration').value || 10);
  if (duration < 3 || duration > 60) { setStatus('每段秒數請填 3 到 60'); return; }
  document.getElementById('phase-help').innerHTML = '<b>' + phaseHelp[phase][0] + '</b><br>' + phaseHelp[phase][1] + '<br><br>現在請按動作，收樣中…';
  post('/api/phase', {phase, duration}).then(x => setStatus(x.message || '收集中'));
}
function updateBars(values) {
  values.forEach((v, i) => {
    const bar = document.getElementById('bar' + i);
    const width = Math.min(Math.abs(v) / 3 * 50, 50);
    bar.style.width = width + '%'; bar.style.left = v >= 0 ? '50%' : (50 - width) + '%';
    bar.classList.toggle('negative', v < 0);
    document.getElementById('value' + i).textContent = (v >= 0 ? '+' : '') + v.toFixed(3);
  });
}
function drawOdomPath(paths, startYawDeg, currentYawDeg) {
  const canvas = document.getElementById('odom-canvas');
  const ctx = canvas.getContext('2d');
  const width = canvas.width, height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#0f172a'; ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = '#334155'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(width / 2, 0); ctx.lineTo(width / 2, height); ctx.moveTo(0, height / 2); ctx.lineTo(width, height / 2); ctx.stroke();
  const tracks = [
    {key: 'visual', label: 'cuVSLAM', color: '#38bdf8', dash: []},
    {key: 'lidar', label: 'LiDAR odom', color: '#fb923c', dash: [8, 5]},
    {key: 'ekf_shadow', label: 'EKF shadow', color: '#a78bfa', dash: []},
  ].map(track => ({...track, points: Array.isArray(paths && paths[track.key]) ? paths[track.key] : []}))
    .filter(track => track.points.length > 0);
  const allPoints = tracks.flatMap(track => track.points);
  if (!allPoints.length) {
    ctx.fillStyle = '#94a3b8'; ctx.font = '15px ui-monospace, monospace';
    ctx.fillText('等待三個 odom source', 18, 28);
    return;
  }
  const xs = allPoints.map(p => p[0]), ys = allPoints.map(p => p[1]);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
  const span = Math.max(maxX - minX, maxY - minY, 0.4);
  const pad = 35, scale = Math.min((width - 2 * pad) / span, (height - 2 * pad) / span);
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
  const map = p => [width / 2 + (p[0] - cx) * scale, height / 2 - (p[1] - cy) * scale];
  tracks.forEach(track => {
    ctx.strokeStyle = track.color; ctx.lineWidth = track.key === 'visual' ? 3 : 2;
    ctx.setLineDash(track.dash); ctx.beginPath();
    track.points.forEach((p, i) => { const q = map(p); if (i === 0) ctx.moveTo(q[0], q[1]); else ctx.lineTo(q[0], q[1]); });
    ctx.stroke(); ctx.setLineDash([]);
    const end = map(track.points[track.points.length - 1]);
    ctx.fillStyle = track.color; ctx.beginPath(); ctx.arc(end[0], end[1], track.key === 'visual' ? 6 : 4, 0, Math.PI * 2); ctx.fill();
  });
  const visual = tracks.find(track => track.key === 'visual');
  const visualPoints = visual ? visual.points : allPoints;
  const start = map(visualPoints[0]), end = map(visualPoints[visualPoints.length - 1]);
  ctx.fillStyle = '#c4b5fd'; ctx.beginPath(); ctx.arc(start[0], start[1], 5, 0, Math.PI * 2); ctx.fill();
  function drawHeading(origin, yawDeg, color) {
    const angle = Number(yawDeg || 0) * Math.PI / 180;
    const length = 34;
    const tip = [origin[0] + Math.cos(angle) * length, origin[1] - Math.sin(angle) * length];
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(origin[0], origin[1]); ctx.lineTo(tip[0], tip[1]); ctx.stroke();
    const head = 8;
    ctx.beginPath();
    ctx.moveTo(tip[0], tip[1]);
    ctx.lineTo(tip[0] - Math.cos(angle - 0.55) * head, tip[1] + Math.sin(angle - 0.55) * head);
    ctx.lineTo(tip[0] - Math.cos(angle + 0.55) * head, tip[1] + Math.sin(angle + 0.55) * head);
    ctx.closePath(); ctx.fill();
  }
  drawHeading(start, startYawDeg, '#c4b5fd');
  drawHeading(end, currentYawDeg, '#fb923c');
  ctx.fillStyle = '#cbd5e1'; ctx.font = '13px ui-monospace, monospace';
  if (Math.hypot(start[0] - end[0], start[1] - end[1]) < 28) {
    ctx.fillText('start / current', end[0] + 10, end[1] - 10);
  } else {
    ctx.fillText('start', start[0] + 8, start[1] - 8);
    ctx.fillText('current', end[0] + 8, end[1] - 8);
  }
  let legendX = 16;
  tracks.forEach(track => {
    ctx.strokeStyle = track.color; ctx.lineWidth = 3; ctx.setLineDash(track.dash);
    ctx.beginPath(); ctx.moveTo(legendX, 18); ctx.lineTo(legendX + 18, 18); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = '#cbd5e1'; ctx.fillText(track.label, legendX + 24, 22);
    legendX += 24 + ctx.measureText(track.label).width + 18;
  });
}
let mappingMapCache = {key: '', cells: null};
function decodeOccupancy(data, key) {
  if (!data) return null;
  if (mappingMapCache.key === key && mappingMapCache.cells) return mappingMapCache.cells;
  try {
    const raw = atob(data);
    const cells = new Uint8Array(raw.length);
    for (let i = 0; i < raw.length; i++) cells[i] = raw.charCodeAt(i);
    mappingMapCache = {key, cells};
    return cells;
  } catch (_error) {
    return null;
  }
}
function localizationOverlap(point, grid, cells) {
  const width = Number(grid.width || 0), height = Number(grid.height || 0), resolution = Number(grid.resolution || 0);
  if (!cells || width < 1 || height < 1 || resolution <= 0 || !Array.isArray(point) || point.length < 2) return 'unknown';
  const origin = Array.isArray(grid.origin) ? grid.origin : [0, 0, 0];
  const yaw = Number(origin[2] || 0) * Math.PI / 180;
  const dx = Number(point[0]) - Number(origin[0] || 0), dy = Number(point[1]) - Number(origin[1] || 0);
  const localX = Math.cos(yaw) * dx + Math.sin(yaw) * dy;
  const localY = -Math.sin(yaw) * dx + Math.cos(yaw) * dy;
  const column = Math.floor(localX / resolution), row = Math.floor(localY / resolution);
  const radius = Math.ceil(0.30 / resolution);
  let nearest = Infinity;
  for (let r = Math.max(0, row - radius); r <= Math.min(height - 1, row + radius); r++) {
    for (let c = Math.max(0, column - radius); c <= Math.min(width - 1, column + radius); c++) {
      if (cells[r * width + c] < 65) continue;
      const distance = Math.hypot(localX - (c + 0.5) * resolution, localY - (r + 0.5) * resolution);
      nearest = Math.min(nearest, distance);
    }
  }
  return nearest <= 0.15 ? 'near' : (nearest <= 0.30 ? 'review' : 'unmatched');
}
function drawMapping(s, targetId = 'mapping-canvas', robotOverride = null, localization = false) {
  const canvas = document.getElementById(targetId);
  const ctx = canvas.getContext('2d');
  const width = canvas.width, height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#0b1120'; ctx.fillRect(0, 0, width, height);
  const grid = s.mapping_map || {};
  const gridWidth = Number(grid.width || 0), gridHeight = Number(grid.height || 0);
  const resolution = Number(grid.resolution || 0);
  const origin = Array.isArray(grid.origin) ? grid.origin : [0, 0, 0];
  const originYaw = Number(origin[2] || 0) * Math.PI / 180;
  const path = Array.isArray(s.mapping_path) ? s.mapping_path : [];
  const scan = s.lidar_scan && Array.isArray(s.lidar_scan.points) ? s.lidar_scan.points : [];
  const robot = Array.isArray(robotOverride) ? robotOverride : (Array.isArray(s.mapping_robot_pose) ? s.mapping_robot_pose : []);
  const worldFromCell = (x, y) => [
    Number(origin[0] || 0) + Math.cos(originYaw) * x - Math.sin(originYaw) * y,
    Number(origin[1] || 0) + Math.sin(originYaw) * x + Math.cos(originYaw) * y,
  ];
  const bounds = [];
  const addBounds = p => { if (Array.isArray(p) && p.length >= 2 && p.every(Number.isFinite)) bounds.push(p); };
  if (gridWidth > 0 && gridHeight > 0 && resolution > 0) {
    addBounds(worldFromCell(0, 0));
    addBounds(worldFromCell(gridWidth * resolution, 0));
    addBounds(worldFromCell(gridWidth * resolution, gridHeight * resolution));
    addBounds(worldFromCell(0, gridHeight * resolution));
  }
  path.forEach(addBounds); scan.forEach(addBounds); addBounds(robot);
  if (!bounds.length) {
    ctx.fillStyle = '#94a3b8'; ctx.font = '15px ui-monospace, monospace';
    ctx.fillText(s.mapping_status || '等待 Jetson 發送 /map、/scan 與 TF', 22, 30);
    return;
  }
  const xs = bounds.map(p => p[0]), ys = bounds.map(p => p[1]);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
  const span = Math.max(maxX - minX, maxY - minY, 0.4);
  const pad = 32, scale = Math.min((width - 2 * pad) / span, (height - 2 * pad) / span);
  const centerX = (minX + maxX) / 2, centerY = (minY + maxY) / 2;
  const toCanvas = p => [width / 2 + (p[0] - centerX) * scale, height / 2 - (p[1] - centerY) * scale];
  const cells = decodeOccupancy(grid.data_b64, `${grid.revision || 0}:${grid.data_b64 ? grid.data_b64.length : 0}`);
  if (cells && gridWidth > 0 && gridHeight > 0 && resolution > 0) {
    const rotated = Math.abs(originYaw) > 0.0001;
    for (let row = 0; row < gridHeight; row++) {
      for (let column = 0; column < gridWidth; column++) {
        const value = cells[row * gridWidth + column];
        if (value === undefined || value === 255) continue;
        if (value >= 65) ctx.fillStyle = '#f8fafc';
        else if (value > 0) ctx.fillStyle = 'rgba(203,213,225,0.60)';
        else ctx.fillStyle = 'rgba(100,116,139,0.22)';
        const p1 = toCanvas(worldFromCell(column * resolution, row * resolution));
        const p2 = toCanvas(worldFromCell((column + 1) * resolution, row * resolution));
        const p3 = toCanvas(worldFromCell((column + 1) * resolution, (row + 1) * resolution));
        const p4 = toCanvas(worldFromCell(column * resolution, (row + 1) * resolution));
        if (!rotated) {
          ctx.fillRect(p1[0], p4[1], p2[0] - p1[0], p1[1] - p4[1]);
        } else {
          ctx.beginPath(); ctx.moveTo(p1[0], p1[1]);
          ctx.lineTo(p2[0], p2[1]); ctx.lineTo(p3[0], p3[1]); ctx.lineTo(p4[0], p4[1]);
          ctx.closePath(); ctx.fill();
        }
      }
    }
  }
  ctx.strokeStyle = '#64748b'; ctx.lineWidth = 1;
  const axisOrigin = toCanvas([0, 0]);
  ctx.beginPath(); ctx.moveTo(axisOrigin[0], 0); ctx.lineTo(axisOrigin[0], height);
  ctx.moveTo(0, axisOrigin[1]); ctx.lineTo(width, axisOrigin[1]); ctx.stroke();
  function drawPolyline(points, color, lineWidth) {
    if (!points || points.length < 1) return;
    ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.beginPath();
    points.forEach((point, index) => {
      const q = toCanvas(point); if (index === 0) ctx.moveTo(q[0], q[1]); else ctx.lineTo(q[0], q[1]);
    });
    ctx.stroke();
  }
  drawPolyline(path, '#38bdf8', 3);
  ctx.fillStyle = '#fb923c';
  scan.forEach(point => {
    const q = toCanvas(point);
    const overlap = localization ? localizationOverlap(point, grid, cells) : 'scan';
    ctx.fillStyle = overlap === 'near' ? '#22c55e' : (overlap === 'review' ? '#facc15' : (overlap === 'unmatched' ? '#ef4444' : '#fb923c'));
    ctx.fillRect(q[0] - 1.5, q[1] - 1.5, 3, 3);
  });
  if (path.length) {
    const start = toCanvas(path[0]); ctx.fillStyle = '#a78bfa';
    ctx.beginPath(); ctx.arc(start[0], start[1], 5, 0, Math.PI * 2); ctx.fill();
  }
  if (robot.length >= 2 && Number.isFinite(robot[0]) && Number.isFinite(robot[1])) {
    const current = toCanvas(robot); const yaw = Number(robot[2] || 0) * Math.PI / 180;
    ctx.fillStyle = '#f97316'; ctx.beginPath(); ctx.arc(current[0], current[1], 7, 0, Math.PI * 2); ctx.fill();
    const tip = [current[0] + Math.cos(yaw) * 34, current[1] - Math.sin(yaw) * 34];
    ctx.strokeStyle = '#fb923c'; ctx.lineWidth = 3; ctx.beginPath();
    ctx.moveTo(current[0], current[1]); ctx.lineTo(tip[0], tip[1]); ctx.stroke();
  }
  ctx.fillStyle = '#cbd5e1'; ctx.font = '13px ui-monospace, monospace';
  ctx.fillText(localization ? (s.localization_status || '定位測試區') : (s.mapping_status || '2D LiDAR map'), 16, 23);
  if (localization) {
    ctx.fillStyle = '#22c55e'; ctx.fillRect(16, height - 24, 8, 8); ctx.fillText('≤0.15 m', 30, height - 19);
    ctx.fillStyle = '#facc15'; ctx.fillRect(96, height - 24, 8, 8); ctx.fillText('≤0.30 m', 110, height - 19);
    ctx.fillStyle = '#ef4444'; ctx.fillRect(176, height - 24, 8, 8); ctx.fillText('unmatched', 190, height - 19);
    ctx.fillStyle = '#ef4444'; ctx.beginPath(); ctx.arc(292, height - 20, 4, 0, Math.PI * 2); ctx.fill(); ctx.fillText('AMCL', 302, height - 19);
  } else {
    ctx.fillStyle = '#38bdf8'; ctx.fillRect(16, height - 24, 12, 3); ctx.fillText('cuVSLAM path', 34, height - 19);
    ctx.fillStyle = '#fb923c'; ctx.fillRect(142, height - 24, 8, 8); ctx.fillText('/scan', 158, height - 19);
    ctx.fillStyle = '#f8fafc'; ctx.fillRect(214, height - 24, 8, 8); ctx.fillText('occupied', 230, height - 19);
  }
}
function updateAssessment(a) {
  const status = a || {};
  const className = status.overall === 'PASS' ? 'quality-good' : (status.overall === 'REVIEW' ? 'quality-warn' : (status.overall === 'FAIL' ? 'quality-bad' : 'r2-note'));
  const score = status.score === null || status.score === undefined ? '—' : `${Number(status.score).toFixed(0)} / 100`;
  const scoreEl = document.getElementById('assessment-score');
  scoreEl.textContent = score;
  scoreEl.className = `assessment-score ${className}`;
  const statusEl = document.getElementById('assessment-status');
  statusEl.textContent = status.label || '尚未連線';
  statusEl.className = `r2-note ${className}`;
  document.getElementById('assessment-vo').textContent = status.vo_valid_ratio === null || status.vo_valid_ratio === undefined ? '—' : `${Number(status.vo_valid_ratio).toFixed(1)}%`;
  document.getElementById('assessment-rate').textContent = status.odom_rate_hz === null || status.odom_rate_hz === undefined ? '—' : `${Number(status.odom_rate_hz).toFixed(1)} Hz`;
  document.getElementById('assessment-gap').textContent = status.image_gap_p95_ms === null || status.image_gap_p95_ms === undefined ? '—' : `${Number(status.image_gap_p95_ms).toFixed(1)} ms`;
  document.getElementById('assessment-stereo-sync').textContent = status.stereo_sync_p95_us === null || status.stereo_sync_p95_us === undefined ? '—' : `${Number(status.stereo_sync_p95_us).toFixed(1)} µs`;
  document.getElementById('assessment-clipping').textContent = status.clipping_pct === null || status.clipping_pct === undefined ? '—' : `${Number(status.clipping_pct).toFixed(1)}%`;
  document.getElementById('assessment-path').textContent = `${Number(status.path_points || 0).toLocaleString()} points`;
  const jitter = status.turn_jitter || {};
  const jitterStatus = jitter.status || '—';
  const jitterFlips = Number(jitter.sign_flips || 0);
  const jitterP95 = Number(jitter.yaw_rate_p95_deg_s || 0);
  document.getElementById('assessment-jitter').textContent = jitterStatus === '—'
    ? '—'
    : `${jitterStatus} · ${jitterFlips} flips · P95 ${jitterP95.toFixed(1)}°/s`;
  const translation = status.turn_translation || {};
  const translationStatus = translation.status || '—';
  const translationCandidate = translation.candidate_status || '—';
  const translationMax = Number(translation.max_xy_displacement_m || 0);
  const translationYaw = Number(translation.yaw_delta_deg || 0);
  const worst = status.worst_geometry || {};
  const worstGate = worst.gate || status.geometry_gate || '—';
  const worstMax = Number(worst.max_xy_displacement_m || 0);
  const worstYaw = Number(worst.yaw_delta_deg || 0);
  document.getElementById('assessment-turn-translation').textContent = translationStatus === '—'
    ? '—'
    : `${translationStatus} · 候選 ${translationCandidate} · max XY ${translationMax.toFixed(3)} m · yaw ${translationYaw.toFixed(1)}° · 整段最差 ${worstGate} / ${worstMax.toFixed(3)} m / ${worstYaw.toFixed(1)}°`;
  document.getElementById('assessment-warning').textContent = (status.warnings || []).join(' · ') || '—';
  const source = status.vo_ratio_source || 'unavailable';
  const samples = Number(status.vo_ratio_samples || 0);
  const sourceText = source === 'raw /visual_slam/status samples'
    ? `raw /visual_slam/status · ${samples} samples`
    : (source === 'GUI telemetry fallback'
      ? `GUI telemetry fallback · ${samples} snapshots（暫不作正式判定）`
      : source);
  document.getElementById('assessment-confidence').textContent = `VO 統計來源：${sourceText}`;
}
function refresh() {
  fetch('/api/state').then(r => r.json()).then(s => {
    setStatus(s.status || '');
    document.getElementById('live').textContent = `gx ${s.gyro[0] >= 0 ? '+' : ''}${s.gyro[0].toFixed(3)} · gy ${s.gyro[1] >= 0 ? '+' : ''}${s.gyro[1].toFixed(3)} · gz ${s.gyro[2] >= 0 ? '+' : ''}${s.gyro[2].toFixed(3)} rad/s`;
    document.getElementById('peak').textContent = `peak ${Math.max(...s.peaks).toFixed(3)} · dominant axis ${s.dominant}`;
    document.getElementById('elapsed').textContent = `elapsed ${s.elapsed.toFixed(1)} s`;
    updateBars(s.gyro);
    document.getElementById('logs').textContent = s.logs.join('\n');
    document.getElementById('mapping-summary').textContent = s.mapping_summary || '尚未有 R2 建圖資料';
    document.getElementById('results').textContent = s.results.join('\n\n');
    document.getElementById('r2-status').textContent = s.r2_status || 'R2 尚未啟動';
    document.getElementById('localization-status').textContent = s.localization_status || '定位尚未啟動';
    document.getElementById('rviz-status').textContent = s.rviz_status || 'RViz 尚未開啟；按鈕會自動開啟可操控的 macOS 螢幕共享';
    document.getElementById('r2-record-status').textContent = s.r2_recording_status || '尚未錄製建圖資料';
    document.getElementById('r2-download-status').textContent = s.r2_download_status || '輕量模式下載：CameraInfo、IMU、TF、odom、VO pose、SLAM path、landmarks cloud 與 GUI 摘要；完整模式才包含原始 IR 影像。';
    const p = s.odom_position || [0, 0, 0];
    document.getElementById('odom-x').textContent = `${p[0] >= 0 ? '+' : ''}${p[0].toFixed(3)} m`;
    document.getElementById('odom-y').textContent = `${p[1] >= 0 ? '+' : ''}${p[1].toFixed(3)} m`;
    document.getElementById('odom-z').textContent = `${p[2] >= 0 ? '+' : ''}${p[2].toFixed(3)} m`;
    document.getElementById('odom-yaw').textContent = `${s.odom_yaw_deg >= 0 ? '+' : ''}${s.odom_yaw_deg.toFixed(2)}°`;
    document.getElementById('odom-rate').textContent = `${s.odom_rate_hz.toFixed(1)} Hz`;
    const telemetryAge = Number(s.telemetry_last_data_age_s || 0);
    const telemetryText = s.telemetry_supervisor_running
      ? (telemetryAge <= 5.0 ? `連線中 · 重連 ${Number(s.telemetry_reconnects || 0)} 次` : `重連中 · ${Number(s.telemetry_reconnects || 0)} 次`)
      : '未連線';
    document.getElementById('odom-telemetry').textContent = telemetryText;
    document.getElementById('odom-state').textContent = s.odom_vo_state;
    document.getElementById('motion-status').textContent = s.motion_status || '尚未記錄';
    document.getElementById('motion-duration').textContent = `${Number(s.motion_duration_s || 0).toFixed(1)} s`;
    document.getElementById('motion-displacement').textContent = `${Number(s.motion_displacement_m || 0).toFixed(3)} m`;
    document.getElementById('motion-path-length').textContent = `${Number(s.motion_path_length_m || 0).toFixed(3)} m`;
    document.getElementById('motion-max').textContent = `${Number(s.motion_max_displacement_m || 0).toFixed(3)} m`;
    document.getElementById('motion-yaw-delta').textContent = `${Number(s.run_yaw_delta_deg || 0).toFixed(1)}°`;
    document.getElementById('motion-samples').textContent = `${Number(s.motion_samples || 0)} / ${Number(s.odom_gap_ms || 0).toFixed(1)} ms`;
    const q = s.image_quality || {};
    const l = q.left || {}, r = q.right || {};
    const sync = q.stereo_sync || {};
    const lw = l.window || {}, rw = r.window || {};
    const metadata = q.metadata || {};
    const ml = (metadata.left || {}).window || {}, mr = (metadata.right || {}).window || {};
    const latestFlags = q.flags || [];
    const qflag = document.getElementById('quality-flag');
    qflag.textContent = latestFlags.length ? latestFlags.join(' · ') : (q.connected ? '影像品質正常範圍' : '尚未連線');
    qflag.className = latestFlags.length ? 'quality-warn' : (q.connected ? 'quality-good' : 'r2-note');
    document.getElementById('quality-left-level').textContent = `${Number(lw.mean || 0).toFixed(1)} / ${Number(lw.std || 0).toFixed(1)}`;
    document.getElementById('quality-right-level').textContent = `${Number(rw.mean || 0).toFixed(1)} / ${Number(rw.std || 0).toFixed(1)}`;
    document.getElementById('quality-clipping').textContent = `${Number(lw.dark_pct || 0).toFixed(1)} / ${Number(lw.bright_pct || 0).toFixed(1)}%`;
    document.getElementById('quality-gradient').textContent = `${Number(lw.gradient || 0).toFixed(2)} / ${Number(rw.gradient || 0).toFixed(2)}`;
    document.getElementById('quality-rate').textContent = `${Number(l.rate_hz || 0).toFixed(1)} / ${Number(r.rate_hz || 0).toFixed(1)} Hz`;
    document.getElementById('quality-gap').textContent = `${Number(l.gap_p95_ms || 0).toFixed(1)} / ${Number(r.gap_p95_ms || 0).toFixed(1)} ms`;
    document.getElementById('quality-stereo-sync').textContent = Number(sync.samples || 0) > 0 ? `${Number(sync.p95_us || 0).toFixed(1)} µs` : '—';
    document.getElementById('quality-exposure').textContent = ml.actual_exposure !== undefined && mr.actual_exposure !== undefined
      ? `${Number(ml.actual_exposure).toFixed(0)} / ${Number(mr.actual_exposure).toFixed(0)} µs` : '—';
    document.getElementById('quality-gain').textContent = ml.gain_level !== undefined && mr.gain_level !== undefined
      ? `${Number(ml.gain_level).toFixed(1)} / ${Number(mr.gain_level).toFixed(1)}` : '—';
    const assessment = s.quality_assessment || {};
    updateAssessment(assessment);
    const estimator = assessment.estimator || {};
    const sources = s.estimator_sources || {};
    const lidarSource = sources.lidar || {};
    const ekfSource = sources.ekf_shadow || {};
    document.getElementById('assessment-estimator').textContent = `${lidarSource.status || 'WAIT'} / ${ekfSource.status || 'WAIT'} · ${estimator.gate || 'PENDING'}`;
    document.getElementById('assessment-estimator-delta').textContent = estimator.status === 'OK'
      ? `${Number(estimator.delta_xy_m || 0).toFixed(3)} m / ${Number(estimator.delta_yaw_deg || 0).toFixed(1)}°`
      : '—';
    const ekfInternal = assessment.ekf_internal || {};
    document.getElementById('assessment-ekf-internal').textContent = ekfInternal.available
      ? `${ekfInternal.processed || '—'} / ${ekfInternal.rejected || '—'} · ${ekfInternal.status || 'UNKNOWN'}`
      : 'WAIT · /diagnostics';
    drawOdomPath(s.estimator_paths || {visual: s.odom_path || []}, Number(s.run_start_yaw_deg || 0), Number(s.odom_yaw_deg || 0));
    const mappingMap = s.mapping_map || {};
    document.getElementById('mapping-status').textContent = s.mapping_status || '尚未收到 /map';
    document.getElementById('mapping-map-size').textContent = Number(mappingMap.width || 0) > 0
      ? `${Number(mappingMap.width)} × ${Number(mappingMap.height)} · ${Number(mappingMap.resolution || 0).toFixed(3)} m`
      : '—';
    document.getElementById('mapping-map-frame').textContent = mappingMap.frame_id || '—';
    const scan = s.lidar_scan || {};
    const scanTiming = scan.timing || {};
    const displayDelay = Number(scanTiming.display_delay_ms);
    const displayDelayText = scanTiming.display_mode === 'timestamped_delayed' && Number.isFinite(displayDelay)
      ? ` · delay ${displayDelay.toFixed(0)} ms` : '';
    document.getElementById('mapping-scan-status').textContent = scan.status
      ? `${scan.status} · ${Number(scan.display_points || 0)} / ${Number(scan.source_points || 0)}${displayDelayText}` : '—';
    const alignment = scan.alignment || {};
    document.getElementById('mapping-alignment-status').textContent = alignment.status === 'MEASURED'
      ? `${Number(alignment.far_ratio || 0).toFixed(0)}% @ 0.30m · ${alignment.best_orientation || '—'}`
      : (alignment.status || '—');
    document.getElementById('mapping-path-points').textContent = Number((s.mapping_path || []).length).toLocaleString();
    drawMapping(s);
    const localizationPose = s.localization_pose || {};
    document.getElementById('localization-map-size').textContent = Number(mappingMap.width || 0) > 0
      ? `${Number(mappingMap.width)} × ${Number(mappingMap.height)} · ${Number(mappingMap.resolution || 0).toFixed(3)} m`
      : 'WAIT /map';
    document.getElementById('localization-map-frame').textContent = mappingMap.frame_id
      ? `${mappingMap.frame_id} · ${Number(mappingMap.resolution || 0).toFixed(3)} m`
      : 'WAIT /map';
    document.getElementById('localization-pose-status').textContent = localizationPose.available
      ? `${localizationPose.status || 'OK'} · ${localizationPose.frame_id || 'map'}`
      : (localizationPose.status || 'WAIT_AMCL');
    const hasLocalizationPose = localizationPose.available
      && Number.isFinite(Number(localizationPose.x_m)) && Number.isFinite(Number(localizationPose.y_m));
    document.getElementById('localization-pose').textContent = hasLocalizationPose
      ? `x ${Number(localizationPose.x_m).toFixed(3)} · y ${Number(localizationPose.y_m).toFixed(3)} · yaw ${Number(localizationPose.yaw_deg || 0).toFixed(1)}°`
      : 'WAIT_AMCL';
    document.getElementById('localization-alignment').textContent = alignment.status === 'MEASURED'
      ? `${Number(alignment.near_ratio || 0).toFixed(0)}% / ${Number(alignment.far_ratio || 0).toFixed(0)}%`
      : (alignment.status || 'WAIT');
    document.getElementById('localization-scan-status').textContent = scan.status
      ? `${scan.status} · ${Number(scan.display_points || 0)} pts`
      : 'WAIT /scan';
    const localizationRobot = hasLocalizationPose
      ? [Number(localizationPose.x_m), Number(localizationPose.y_m), Number(localizationPose.yaw_deg || 0)]
      : [];
    drawMapping({
      ...s,
      localization_status: s.localization_status || '定位測試區等待資料',
    }, 'localization-canvas', localizationRobot, true);
    document.querySelectorAll('[data-phase]').forEach(b => b.disabled = !s.rig_running || !!s.phase);
    document.getElementById('start').disabled = s.rig_running;
    document.getElementById('localization-start').disabled = s.r2_running || !!s.r2_busy || s.localization_running || !!s.localization_busy;
    document.getElementById('localization-stop').disabled = !s.localization_running && !s.localization_busy;
    document.getElementById('r2-start').disabled = s.r2_running || !!s.r2_busy || s.localization_running || !!s.localization_busy;
    document.getElementById('r2-stop').disabled = !!s.r2_busy;
    document.getElementById('r2-record-start').disabled = !s.r2_running || !!s.r2_recording;
    document.getElementById('r2-record-stop').disabled = !s.r2_recording;
    document.getElementById('r2-download').disabled = !s.r2_recording_name || !!s.r2_recording;
    document.getElementById('rviz-open').disabled = !!s.rviz_opening;
    document.getElementById('motion-start').disabled = !s.r2_running || !!s.motion_recording;
    document.getElementById('motion-stop').disabled = !s.motion_recording;
  }).catch(() => setStatus('本機 GUI server 連線中斷'));
}
setInterval(refresh, 500); refresh();
restoreSections();
</script>
</body>
</html>"""


class State:
    def __init__(self) -> None:
        # A few worker completion paths append a diagnostic while updating
        # state.  Re-entrant locking keeps those paths from self-deadlocking
        # the HTTP /api/state handler.
        self.lock = threading.RLock()
        self.status = "尚未啟動測試容器"
        self.rig_running = False
        self.phase = ""
        self.output_name = ""
        self.gyro = [0.0, 0.0, 0.0]
        self.peaks = [0.0, 0.0, 0.0]
        self.dominant: int | str = "—"
        self.elapsed = 0.0
        self.logs: list[str] = []
        self.results: list[str] = []
        self.phase_process: subprocess.Popen[str] | None = None
        self.r2_status = "R2 尚未啟動"
        self.r2_running = False
        self.r2_busy = False
        self.r2_mode = "imu"
        self.r2_mapping = "mapping"
        self.r2_profile = "640x360x30"
        self.r2_lighting = "official"
        self.r2_emitter_override = "on"
        self.r2_diagnostics = "standard"
        self.r2_estimator = "cuvslam"
        self.r2_scene_label = "scene_unknown"
        self.r2_motion_label = "other"
        self.r2_started_at = ""
        self.last_mapping_summary = ""
        self.run_trace_file = ""
        self.run_trace_records = 0
        self.run_session_file = ""
        self.r2_recording = False
        self.r2_recording_name = ""
        self.r2_recording_mode = "light"
        self.r2_recording_status = "尚未錄製建圖資料"
        self.r2_download_status = "輕量模式包含 IMU、TF、odom、VO、SLAM path、landmarks、pose graph 與 loop closure；特徵診斷開啟時才有 observations；完整模式另外包含原始 IR 影像。"
        self.localization_status = "定位尚未啟動"
        self.localization_running = False
        self.localization_busy = False
        self.localization_started_at = ""
        self.localization_pose: dict[str, object] = {
            "available": False, "status": "WAIT_AMCL", "frame_id": "",
            "x_m": None, "y_m": None, "yaw_deg": None,
            "covariance_x_m2": None, "covariance_y_m2": None,
            "covariance_yaw_rad2": None,
        }
        self.rviz_status = "RViz 尚未開啟；按鈕會自動開啟可操控的 macOS 螢幕共享"
        self.rviz_opening = False
        self.rviz_tunnel_process: subprocess.Popen[str] | None = None
        self.rviz_viewer_process: subprocess.Popen[str] | None = None
        self.telemetry_process: subprocess.Popen[str] | None = None
        self.telemetry_supervisor_running = False
        self.telemetry_mode = "r2"
        self.telemetry_reconnects = 0
        self.telemetry_last_data_monotonic = 0.0
        self.quality_process: subprocess.Popen[str] | None = None
        self.image_quality: dict[str, object] = {"connected": False, "flags": []}
        self.observed_quality_flags: set[str] = set()
        self.last_logged_metadata_signature: tuple[object, ...] | None = None
        self.last_logged_quality_signature: tuple[str, ...] | None = None
        self.last_logged_estimator_signature: tuple[object, ...] | None = None
        self.last_logged_contribution_signature: tuple[object, ...] | None = None
        self.last_logged_ekf_diagnostics_signature: tuple[object, ...] | None = None
        self.last_logged_odom_guard_signature: tuple[object, ...] | None = None
        self.last_logged_covariance_signature: tuple[object, ...] | None = None
        self.last_logged_scan_timing_signature: tuple[object, ...] | None = None
        self.estimator_sources: dict[str, object] = {}
        self.estimator_comparisons: dict[str, object] = {}
        self.estimator_contribution: dict[str, object] = {}
        self.ekf_diagnostics: dict[str, object] = {}
        self.odom_guard_diagnostics: dict[str, object] = {}
        self.estimator_paths: dict[str, list[list[float]]] = {
            "visual": [], "lidar": [], "ekf_shadow": [],
        }
        self.estimator_path_origin: list[float] | None = None
        self.estimator_path_last_samples: dict[str, int] = {}
        self.odom_position = [0.0, 0.0, 0.0]
        self.odom_yaw_deg = 0.0
        self.odom_rate_hz = 0.0
        self.odom_vo_state: int | str = "—"
        self.odom_vo_valid_ratio: float | None = None
        self.odom_vo_ratio_source = "unavailable"
        self.odom_vo_ratio_samples = 0
        self.odom_vo_valid_samples = 0
        self.odom_gap_p95_ms = 0.0
        self.odom_rate_basis = "unavailable"
        self.odom_timestamp_span_s = 0.0
        self.turn_jitter_status = "未偵測"
        self.turn_jitter_samples = 0
        self.turn_yaw_rate_p95_deg_s = 0.0
        self.turn_sign_flips = 0
        self.turn_flip_ratio = 0.0
        self.turn_translation_status = "未偵測"
        self.turn_translation_candidate_status = "未偵測"
        self.turn_translation_segments = 0
        self.turn_translation_yaw_delta_deg = 0.0
        self.turn_translation_max_xy_m = 0.0
        self.turn_translation_path_m = 0.0
        self.worst_geometry_gate = "未偵測"
        self.worst_geometry_segments = 0
        self.worst_geometry_yaw_delta_deg = 0.0
        self.worst_geometry_max_xy_m = 0.0
        self.worst_geometry_path_m = 0.0
        self.run_start_yaw_deg = 0.0
        self.run_yaw_delta_deg = 0.0
        self.run_yaw_unwrapped_deg = 0.0
        self.turn_history: list[dict[str, object]] = []
        self.odom_telemetry_count = 0
        self.odom_telemetry_valid_count = 0
        self.odom_path_length_offset_m = 0.0
        self.odom_samples_offset = 0
        self.odom_vo_status_offset = 0
        self.odom_vo_valid_offset = 0
        self.remote_odom_samples = 0
        self.remote_path_length_m = 0.0
        self.remote_vo_status_samples = 0
        self.remote_vo_valid_samples = 0
        self.odom_path: list[list[float]] = []
        self.odom_path_length_m = 0.0
        self.odom_samples = 0
        self.odom_gap_ms = 0.0
        self.mapping_map: dict[str, object] = {}
        self.mapping_status = "尚未收到 /map"
        self.mapping_path: list[list[float]] = []
        self.mapping_robot_pose: list[float] = []
        self.lidar_scan: dict[str, object] = {
            "status": "尚未收到 /scan",
            "frame_id": "",
            "points": [],
            "source_points": 0,
            "display_points": 0,
            "alignment": {"status": "WAIT"},
            "timing": {"status": "WAIT"},
        }
        self.motion_recording = False
        self.motion_status = "尚未記錄"
        self.motion_started_at = 0.0
        self.motion_start_position: list[float] | None = None
        self.motion_start_path_length_m: float | None = None
        self.motion_start_samples = 0
        self.motion_duration_s = 0.0
        self.motion_displacement_m = 0.0
        self.motion_path_length_m = 0.0
        self.motion_max_displacement_m = 0.0
        self.motion_samples = 0
        self.motion_quality_samples: list[dict[str, object]] = []
        self.motion_record_file = ""

    def add_log(self, line: str) -> None:
        if not line:
            return
        with self.lock:
            self.logs.append(line)
            self.logs[-1] = f"[{time.strftime('%H:%M:%S')}] {self.logs[-1]}"
            # Keep the process/event history readable without trimming it to
            # a tiny tail.  The copyable mapping summary is separate from
            # this timeline and should not replace it.
            self.logs = self.logs[-5000:]

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            image_quality = dict(self.image_quality)
            image_quality["observed_flags"] = sorted(self.observed_quality_flags)
            return {
                "status": self.status, "rig_running": self.rig_running, "phase": self.phase,
                "gyro": list(self.gyro), "peaks": list(self.peaks), "dominant": self.dominant,
                "elapsed": self.elapsed, "logs": list(self.logs), "results": list(self.results),
                "r2_status": self.r2_status, "r2_running": self.r2_running, "r2_mode": self.r2_mode,
                "r2_busy": self.r2_busy,
                "r2_mapping": self.r2_mapping,
                "r2_profile": self.r2_profile,
                "r2_lighting": self.r2_lighting,
                "r2_emitter_override": self.r2_emitter_override,
                "r2_diagnostics": self.r2_diagnostics,
                "r2_estimator": self.r2_estimator,
                "r2_scene_label": self.r2_scene_label,
                "r2_motion_label": self.r2_motion_label,
                "r2_started_at": self.r2_started_at,
                "run_trace_file": self.run_trace_file,
                "run_trace_records": self.run_trace_records,
                "run_session_file": self.run_session_file,
                "r2_recording": self.r2_recording,
                "r2_recording_name": self.r2_recording_name,
                "r2_recording_mode": self.r2_recording_mode,
                "r2_recording_status": self.r2_recording_status,
                "r2_download_status": self.r2_download_status,
                "localization_status": self.localization_status,
                "localization_running": self.localization_running,
                "localization_busy": self.localization_busy,
                "localization_started_at": self.localization_started_at,
                "localization_pose": dict(self.localization_pose),
                "telemetry_supervisor_running": self.telemetry_supervisor_running,
                "telemetry_reconnects": self.telemetry_reconnects,
                "telemetry_last_data_age_s": (
                    max(0.0, time.monotonic() - self.telemetry_last_data_monotonic)
                    if self.telemetry_last_data_monotonic else 0.0
                ),
                "mapping_summary": self.mapping_summary_text(),
                "rviz_status": self.rviz_status,
                "rviz_opening": self.rviz_opening,
                "image_quality": image_quality,
                "odom_position": list(self.odom_position), "odom_yaw_deg": self.odom_yaw_deg,
                "run_start_yaw_deg": self.run_start_yaw_deg,
                "run_yaw_delta_deg": self.run_yaw_delta_deg,
                "run_yaw_unwrapped_deg": self.run_yaw_unwrapped_deg,
                "odom_rate_hz": self.odom_rate_hz, "odom_vo_state": self.odom_vo_state,
                "odom_vo_valid_ratio": self.odom_vo_valid_ratio,
                "odom_vo_ratio_source": self.odom_vo_ratio_source,
                "odom_vo_ratio_samples": self.odom_vo_ratio_samples,
                "odom_vo_valid_samples": self.odom_vo_valid_samples,
                "odom_gap_p95_ms": self.odom_gap_p95_ms,
                "odom_rate_basis": self.odom_rate_basis,
                "odom_timestamp_span_s": self.odom_timestamp_span_s,
                "turn_jitter": self.turn_jitter_summary(),
                "turn_translation": self.turn_translation_summary(),
                "turn_history": list(self.turn_history),
                "odom_path": list(self.odom_path),
                "odom_path_length_m": self.odom_path_length_m, "odom_samples": self.odom_samples,
                "odom_gap_ms": self.odom_gap_ms,
                "mapping_map": dict(self.mapping_map),
                "mapping_status": self.mapping_status,
                "mapping_path": list(self.mapping_path),
                "mapping_robot_pose": list(self.mapping_robot_pose),
                "lidar_scan": dict(self.lidar_scan),
                "estimator_sources": dict(self.estimator_sources),
                "estimator_comparisons": dict(self.estimator_comparisons),
                "estimator_contribution": dict(self.estimator_contribution),
                "ekf_diagnostics": dict(self.ekf_diagnostics),
                "odom_guard_diagnostics": dict(self.odom_guard_diagnostics),
                "estimator_paths": {
                    name: list(points) for name, points in self.estimator_paths.items()
                },
                "quality_assessment": self.quality_assessment(),
                "motion_recording": self.motion_recording, "motion_status": self.motion_status,
                "motion_duration_s": (time.monotonic() - self.motion_started_at
                                       if self.motion_recording and self.motion_started_at else self.motion_duration_s),
                "motion_displacement_m": self.motion_displacement_m,
                "motion_path_length_m": self.motion_path_length_m,
                "motion_max_displacement_m": self.motion_max_displacement_m,
                "motion_samples": self.motion_samples,
                "motion_quality_samples": len(self.motion_quality_samples),
                "motion_record_file": self.motion_record_file,
            }

    def mapping_summary_text(self) -> str:
        """Build a copyable, report-friendly summary of the current R2 run."""
        if self.odom_samples < 1 and self.last_mapping_summary:
            return self.last_mapping_summary
        if self.odom_samples < 1:
            return "尚未有 R2 建圖資料"

        def number(value: object, default: float = 0.0) -> float:
            try:
                return float(value)  # type: ignore[arg-type]
            except (TypeError, ValueError):
                return default

        position = self.odom_position
        path = self.odom_path
        start_xy = path[0] if path else [0.0, 0.0]
        closure_xy = math.hypot(position[0] - start_xy[0], position[1] - start_xy[1])
        quality = self.image_quality
        left = quality.get("left", {}) if isinstance(quality, dict) else {}
        right = quality.get("right", {}) if isinstance(quality, dict) else {}
        left_window = left.get("window", {}) if isinstance(left, dict) else {}
        right_window = right.get("window", {}) if isinstance(right, dict) else {}
        assessment = self.quality_assessment()
        warnings = assessment.get("warnings", [])
        left_rate = number(left.get("rate_hz")) if isinstance(left, dict) else 0.0
        right_rate = number(right.get("rate_hz")) if isinstance(right, dict) else 0.0
        left_gap = number(left.get("gap_p95_ms")) if isinstance(left, dict) else 0.0
        right_gap = number(right.get("gap_p95_ms")) if isinstance(right, dict) else 0.0
        stereo_sync = quality.get("stereo_sync", {}) if isinstance(quality, dict) else {}
        input_flags = [str(flag) for flag in assessment.get("input_flags", [])]
        current_input_flags = [
            str(flag) for flag in quality.get("flags", [])
        ] if isinstance(quality, dict) and isinstance(quality.get("flags", []), list) else []
        stereo_sync_p95_us = number(stereo_sync.get("p95_us")) if isinstance(stereo_sync, dict) else 0.0
        left_clip = number(left_window.get("bright_pct")) if isinstance(left_window, dict) else 0.0
        right_clip = number(right_window.get("bright_pct")) if isinstance(right_window, dict) else 0.0
        score = assessment.get("score")
        score_text = "—" if score is None else f"{number(score):.1f}"
        vo_value = assessment.get("vo_valid_ratio")
        vo_text = "—" if vo_value is None else f"{number(vo_value):.1f}%"
        vo_source = str(assessment.get("vo_ratio_source") or "unavailable")
        vo_samples = int(assessment.get("vo_ratio_samples") or 0)
        if vo_source == "raw /visual_slam/status samples":
            vo_source_text = f"raw status · {vo_samples} samples"
        elif vo_source == "GUI telemetry fallback":
            vo_source_text = f"GUI fallback · {vo_samples} snapshots（暫不作正式判定）"
        else:
            vo_source_text = vo_source
        rate_basis = self.odom_rate_basis or "unavailable"
        rate_span = self.odom_timestamp_span_s
        rate_text = f"{rate_basis}"
        if rate_span > 0.0:
            rate_text += f" · span {rate_span:.1f} s"
        telemetry_text = (
            "連線中" if self.telemetry_supervisor_running and self.telemetry_process is not None
            else ("重連中" if self.telemetry_supervisor_running else "未連線")
        )
        jitter = self.turn_jitter_summary()
        turn_translation = self.turn_translation_summary()
        estimator = self.estimator_assessment()
        estimator_sources = self.estimator_sources
        lidar_source = estimator_sources.get("lidar", {})
        ekf_source = estimator_sources.get("ekf_shadow", {})
        lidar_status = str(lidar_source.get("status", "WAIT")) if isinstance(lidar_source, dict) else "WAIT"
        ekf_status = str(ekf_source.get("status", "WAIT")) if isinstance(ekf_source, dict) else "WAIT"
        estimator_delta_xy = number(estimator.get("delta_xy_m"))
        estimator_delta_yaw = number(estimator.get("delta_yaw_deg"))
        pull_estimate = self.estimator_contribution.get("pull_estimate", {}) if isinstance(self.estimator_contribution, dict) else {}
        pull_xy = pull_estimate.get("xy", {}) if isinstance(pull_estimate, dict) else {}
        pull_yaw = pull_estimate.get("yaw", {}) if isinstance(pull_estimate, dict) else {}
        internal_ekf = self.ekf_internal_summary()
        covariance = self.estimator_covariance_summary()
        skew = self.estimator_time_skew_summary()
        def percentage(value: object) -> str:
            if value is None:
                return "—"
            try:
                return f"{float(value):.1f}"
            except (TypeError, ValueError):
                return "—"
        internal_processed = internal_ekf.get("processed") or "—"
        internal_rejected = internal_ekf.get("rejected") or "—"

        return "\n".join([
            "=== cuVSLAM R2 建圖分析摘要 ===",
            f"開始時間：{self.r2_started_at or '—'}",
            f"模式：{self.r2_mode} · 工作：{self.r2_mapping} · 影像：{self.r2_profile} · 光照：{self.r2_lighting}",
            f"特徵診斷：{self.r2_diagnostics}（features 才會輸出 observations cloud）",
            f"估計器：{self.r2_estimator} · LiDAR odom={lidar_status} · EKF shadow={ekf_status}",
            f"估計器一致性：{estimator.get('gate', 'PENDING')} · pair={estimator.get('pair') or '—'} · ΔXY={estimator_delta_xy:.3f} m · Δyaw={estimator_delta_yaw:.1f}°",
            f"估計器時間差：visual↔LiDAR={skew.get('visual_vs_lidar', 0.0):.1f} ms · visual↔EKF={skew.get('visual_vs_ekf_shadow', 0.0):.1f} ms · LiDAR↔EKF={skew.get('lidar_vs_ekf_shadow', 0.0):.1f} ms",
            f"EKF 介入程度（軌跡推估）：XY LiDAR={percentage(pull_xy.get('lidar_pct'))}% / cuVSLAM={percentage(pull_xy.get('cuvslam_pct'))}% · yaw LiDAR={percentage(pull_yaw.get('lidar_pct'))}% / cuVSLAM={percentage(pull_yaw.get('cuvslam_pct'))}%",
            f"EKF 內部診斷：{internal_ekf.get('status', 'WAIT')} · processed={internal_processed} · rejected={internal_rejected} · 實際 Kalman gain=未由 ROS 診斷介面提供",
            f"協方差輸入：{covariance.get('status', 'WAIT')} · {'、'.join(str(item) for item in covariance.get('warnings', [])) if covariance.get('warnings') else '未見啟用欄位零值'} · 政策=只標示，不改寫 source、不接管 TF",
            f"場景標籤：{self.r2_scene_label} · 動作標籤：{self.r2_motion_label}",
            f"品質判定：{assessment.get('label', '—')} · 資料流分數：{score_text} / 100",
            f"VO 有效率：{vo_text} · 當前 vo_state：{self.odom_vo_state}",
            f"VO 統計來源：{vo_source_text}",
            f"odom rate：{self.odom_rate_hz:.1f} Hz · odom gap P95：{self.odom_gap_p95_ms:.1f} ms",
            f"odom rate 計算基準：{rate_text}",
            f"odom telemetry：{telemetry_text} · 自動重連 {self.telemetry_reconnects} 次",
            f"轉向抖動診斷：{jitter['status']} · turn samples={jitter['turn_samples']} · yaw rate P95={jitter['yaw_rate_p95_deg_s']:.1f}°/s · sign flips={jitter['sign_flips']}",
            f"整段 yaw Δ：{self.run_yaw_delta_deg:.1f}° · 累積 yaw：{self.run_yaw_unwrapped_deg:+.1f}°",
            f"目前最後轉向片段：{turn_translation['status']} · 原地旋轉候選={turn_translation['candidate_status']} · segments={turn_translation['turn_segments']} · 片段 yaw Δ={turn_translation['yaw_delta_deg']:.1f}° · max XY={turn_translation['max_xy_displacement_m']:.3f} m · turn path={turn_translation['path_length_m']:.3f} m",
            f"整段轉向幾何最差觀測：{assessment.get('geometry_gate', '未偵測')} · segments={self.worst_geometry_segments} · 片段 yaw Δ={self.worst_geometry_yaw_delta_deg:.1f}° · max XY={self.worst_geometry_max_xy_m:.3f} m · turn path={self.worst_geometry_path_m:.3f} m",
            f"轉向片段完整紀錄：{len(self.turn_history)} 段",
            f"影像 rate L/R：{left_rate:.1f} / {right_rate:.1f} Hz · image gap P95 L/R：{left_gap:.1f} / {right_gap:.1f} ms",
            f"左右影像同步 P95：{stereo_sync_p95_us:.1f} µs（建議 ≤100 µs）",
            f"影像 clipped L/R：{left_clip:.1f}% / {right_clip:.1f}%",
            f"影像品質旗標（本次觀測）：{', '.join(input_flags) if input_flags else 'none'} · 目前：{', '.join(current_input_flags) if current_input_flags else 'none'}",
            f"完整路徑點：{len(path)} · 累積路徑：{self.odom_path_length_m:.3f} m",
            f"目前位置：x={position[0]:+.3f} m, y={position[1]:+.3f} m, z={position[2]:+.3f} m",
            f"目前 yaw：{self.odom_yaw_deg:+.2f}° · 起點到目前 XY：{closure_xy:.3f} m",
            f"2D LiDAR map：{self.mapping_status} · frame={self.mapping_map.get('frame_id', '—')} · size={self.mapping_map.get('width', '—')}x{self.mapping_map.get('height', '—')} · resolution={number(self.mapping_map.get('resolution'), 0.0):.3f} m",
            f"2D LiDAR scan：{self.lidar_scan.get('status', '—')} · 顯示 {self.lidar_scan.get('display_points', 0)} / 來源 {self.lidar_scan.get('source_points', 0)} 點 · map frame 路徑點={len(self.mapping_path)}",
            f"LiDAR 時間診斷：{self.lidar_scan_timing_summary()}",
            f"閉環參考：位置 ≤0.10 m Pass / ≤0.20 m Review；yaw ≤5° Pass / ≤10° Review",
            f"目前警告：{' · '.join(str(item) for item in warnings) if warnings else '無'}",
            "資料來源：/visual_slam/tracking/odometry、/lidar/odom、/odometry/filtered_shadow、/visual_slam/status、/map、/scan、TF、RealSense IR image quality",
            f"完整追蹤紀錄：{self.run_trace_file or '—'} · records={self.run_trace_records}",
            f"本次 session：{self.run_session_file or '停止 R2 後產生'}",
            f"修正紀錄：{CORRECTION_LOG_PATH}",
        ])

    def lidar_scan_timing_summary(self) -> str:
        timing = self.lidar_scan.get("timing", {})
        if not isinstance(timing, dict) or timing.get("status") in (None, "WAIT"):
            return "WAIT"

        def value(name: str, suffix: str = "") -> str:
            item = timing.get(name)
            if item is None:
                return "—"
            try:
                return f"{float(item):.3f}{suffix}"
            except (TypeError, ValueError):
                return "—"

        stamp = timing.get("scan_stamp_ns", "—")
        tf_status = str(timing.get("tf_lookup_status", "—"))
        tf_delta = value("tf_lookup_delta_ms")
        latest_delta = value("tf_latest_delta_ms")
        scan_age = value("timestamp_age_ms", " ms")
        scan_end_age = value("scan_end_age_ms", " ms")
        error = str(timing.get("tf_error") or "")
        error_text = f" · error={error}" if error else ""
        display_mode = str(timing.get("display_mode", "—"))
        display_delay = value("display_delay_ms", " ms")
        queue_depth = timing.get("queue_depth", "—")
        queue_drops = timing.get("queue_drops", "—")
        return (
            f"{timing.get('status', '—')} · stamp={stamp} · "
            f"scan_time={value('scan_time_s', ' s')} · "
            f"beam_span={value('beam_span_ms', ' ms')} · "
            f"time_increment={value('time_increment_us', ' µs')} · "
            f"scan_age={scan_age} · scan_end_age={scan_end_age} · "
            f"TF={tf_status} · lookup Δ={tf_delta} ms · latest Δ={latest_delta} ms · "
            f"display={display_mode} · delay={display_delay} · queue={queue_depth} · drops={queue_drops}"
            f"{error_text}"
        )

    def turn_jitter_summary(self) -> dict[str, float | int | str]:
        return {
            "status": self.turn_jitter_status,
            "turn_samples": self.turn_jitter_samples,
            "yaw_rate_p95_deg_s": self.turn_yaw_rate_p95_deg_s,
            "sign_flips": self.turn_sign_flips,
            "flip_ratio": self.turn_flip_ratio,
        }

    def turn_translation_summary(self) -> dict[str, float | int | str]:
        return {
            "status": self.turn_translation_status,
            "candidate_status": self.turn_translation_candidate_status,
            "turn_segments": self.turn_translation_segments,
            "yaw_delta_deg": self.turn_translation_yaw_delta_deg,
            "max_xy_displacement_m": self.turn_translation_max_xy_m,
            "path_length_m": self.turn_translation_path_m,
        }

    def worst_geometry_summary(self) -> dict[str, float | int | str]:
        return {
            "gate": self.worst_geometry_gate,
            "turn_segments": self.worst_geometry_segments,
            "yaw_delta_deg": self.worst_geometry_yaw_delta_deg,
            "max_xy_displacement_m": self.worst_geometry_max_xy_m,
            "path_length_m": self.worst_geometry_path_m,
        }

    def observe_geometry(
        self,
        candidate_status: object,
        turn_segments: object,
        yaw_delta_deg: object,
        max_xy_displacement_m: object,
        path_length_m: object,
    ) -> None:
        """Retain the strongest geometry warning seen during this run.

        The live telemetry is a snapshot and can return to ``未偵測`` after a
        turn ends.  A final assessment must not erase a prior FAIL/REVIEW.
        Callers hold ``self.lock`` while updating the live state.
        """
        gate = str(candidate_status)
        if gate not in GEOMETRY_GATE_RANK or gate in {"未偵測", "PENDING"}:
            return
        try:
            segments = int(turn_segments)
        except (TypeError, ValueError):
            segments = 0
        try:
            yaw_delta = float(yaw_delta_deg)
        except (TypeError, ValueError):
            yaw_delta = 0.0
        try:
            max_xy = float(max_xy_displacement_m)
        except (TypeError, ValueError):
            max_xy = 0.0
        try:
            path_length = float(path_length_m)
        except (TypeError, ValueError):
            path_length = 0.0

        current_key = (GEOMETRY_GATE_RANK[gate], max_xy, abs(yaw_delta))
        worst_key = (
            GEOMETRY_GATE_RANK.get(self.worst_geometry_gate, 0),
            self.worst_geometry_max_xy_m,
            abs(self.worst_geometry_yaw_delta_deg),
        )
        if current_key > worst_key:
            self.worst_geometry_gate = gate
            self.worst_geometry_segments = segments
            self.worst_geometry_yaw_delta_deg = yaw_delta
            self.worst_geometry_max_xy_m = max_xy
            self.worst_geometry_path_m = path_length

    def ekf_internal_summary(self) -> dict[str, object]:
        """Expose only fields actually reported by robot_localization."""
        diagnostics = self.ekf_diagnostics
        if not isinstance(diagnostics, dict) or not diagnostics.get("available"):
            return {
                "status": "WAIT",
                "available": False,
                "processed": None,
                "rejected": None,
                "dropped": None,
                "frequency": None,
                "message": "尚未收到 robot_localization /diagnostics",
            }
        raw_values = diagnostics.get("values", {})
        values = raw_values if isinstance(raw_values, dict) else {}

        def find_value(*terms: str) -> str | None:
            for key, value in values.items():
                lowered = str(key).lower()
                if any(term in lowered for term in terms):
                    return str(value)
            return None

        level = int(diagnostics.get("level", 0))
        status = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}.get(level, "UNKNOWN")
        return {
            "status": status,
            "available": True,
            "name": diagnostics.get("name", ""),
            "processed": find_value("processed", "measurements processed"),
            "rejected": find_value("rejected", "measurements rejected"),
            "dropped": find_value("dropped", "measurements dropped"),
            "ignored": find_value("ignored", "measurements ignored"),
            "frequency": find_value("frequency", "frequency (hz)"),
            "message": diagnostics.get("message", ""),
        }

    def estimator_assessment(self) -> dict[str, object]:
        """Summarize the optional odometry comparison without changing TF."""
        preferred_pairs = ("visual_vs_lidar", "visual_vs_ekf_shadow")
        for pair_name in preferred_pairs:
            comparison = self.estimator_comparisons.get(pair_name, {})
            if not isinstance(comparison, dict):
                continue
            gate = str(comparison.get("gate", "PENDING"))
            if gate in {"PASS", "REVIEW", "FAIL"}:
                return {
                    "status": "OK",
                    "pair": pair_name,
                    "gate": gate,
                    "delta_xy_m": float(comparison.get("delta_xy_m", 0.0)),
                    "delta_yaw_deg": float(comparison.get("delta_yaw_deg", 0.0)),
                "reason": comparison.get("reason", ""),
            }
        return {
            "status": "WAIT",
            "pair": "",
            "gate": "PENDING",
            "delta_xy_m": 0.0,
            "delta_yaw_deg": 0.0,
            "reason": "尚未收到可比較的 LiDAR odom / EKF shadow",
        }

    def estimator_covariance_summary(self) -> dict[str, object]:
        """Summarize covariance fields relevant to the shadow EKF inputs."""
        source_status: dict[str, object] = {}
        warnings: list[str] = []
        for name in ("visual", "lidar", "ekf_shadow"):
            source = self.estimator_sources.get(name, {})
            if not isinstance(source, dict) or source.get("status") == "WAIT":
                source_status[name] = {"pose": "WAIT", "twist": "WAIT", "pose_planar_zero_indices": [], "twist_planar_zero_indices": []}
                continue
            covariance = source.get("covariance", {}) if isinstance(source, dict) else {}
            pose = covariance.get("pose", {}) if isinstance(covariance, dict) else {}
            twist = covariance.get("twist", {}) if isinstance(covariance, dict) else {}
            pose_status = str(pose.get("status", "WAIT")) if isinstance(pose, dict) else "WAIT"
            twist_status = str(twist.get("status", "WAIT")) if isinstance(twist, dict) else "WAIT"
            source_status[name] = {
                "pose": pose_status,
                "twist": twist_status,
                "pose_planar_zero_indices": list(pose.get("planar_zero_indices", [])) if isinstance(pose, dict) else [],
                "twist_planar_zero_indices": list(twist.get("planar_zero_indices", [])) if isinstance(twist, dict) else [],
            }
            if pose_status == "ZERO_FUSED_FIELDS" or twist_status == "ZERO_FUSED_FIELDS":
                warnings.append(f"{name} fused covariance zero")
            if pose_status == "INVALID" or twist_status == "INVALID":
                warnings.append(f"{name} covariance invalid")
        return {
            "status": "WARN" if warnings else ("OK" if source_status else "WAIT"),
            "warnings": warnings,
            "sources": source_status,
            "policy": "只記錄並標示零協方差；目前不改寫 source message，也不讓 shadow 接管 TF",
        }

    def estimator_time_skew_summary(self) -> dict[str, float]:
        result: dict[str, float] = {}
        for name, comparison in self.estimator_comparisons.items():
            if not isinstance(comparison, dict):
                continue
            value = comparison.get("timestamp_skew_ms")
            if value is not None:
                try:
                    result[name] = float(value)
                except (TypeError, ValueError):
                    continue
        return result

    def update_estimator_paths(self, sources: dict[str, object]) -> None:
        """Keep bounded common-odom tracks for visual, LiDAR and EKF shadow."""
        if self.estimator_path_origin is None:
            candidates = ["visual", "lidar", "ekf_shadow"]
            for name in candidates:
                source = sources.get(name, {})
                position = source.get("position", []) if isinstance(source, dict) else []
                if source.get("status") == "OK" and isinstance(position, list) and len(position) >= 2:
                    self.estimator_path_origin = [float(position[0]), float(position[1])]
                    break
        if self.estimator_path_origin is None:
            return
        origin = self.estimator_path_origin
        for name in ("visual", "lidar", "ekf_shadow"):
            source = sources.get(name, {})
            if not isinstance(source, dict) or source.get("status") == "WAIT":
                continue
            try:
                sample_count = int(source.get("samples", 0))
            except (TypeError, ValueError):
                continue
            last_sample = self.estimator_path_last_samples.get(name, 0)
            if sample_count <= last_sample:
                continue
            position = source.get("position", [])
            if not isinstance(position, list) or len(position) < 2:
                continue
            self.estimator_paths.setdefault(name, []).append([
                float(position[0]) - origin[0],
                float(position[1]) - origin[1],
            ])
            self.estimator_paths[name] = self.estimator_paths[name][-1800:]
            self.estimator_path_last_samples[name] = sample_count

    def quality_assessment(self) -> dict[str, object]:
        """Return a conservative, explainable online R2 quality assessment."""
        quality = self.image_quality
        left = quality.get("left", {}) if isinstance(quality, dict) else {}
        right = quality.get("right", {}) if isinstance(quality, dict) else {}
        left_window = left.get("window", {}) if isinstance(left, dict) else {}
        right_window = right.get("window", {}) if isinstance(right, dict) else {}
        stereo_sync = quality.get("stereo_sync", {}) if isinstance(quality, dict) else {}
        current_input_flags = [
            str(flag) for flag in quality.get("flags", [])
        ] if isinstance(quality, dict) and isinstance(quality.get("flags", []), list) else []
        input_flags = sorted(self.observed_quality_flags or set(current_input_flags))
        current_geometry_gate = self.turn_translation_candidate_status
        geometry_gate = self.worst_geometry_gate
        if GEOMETRY_GATE_RANK.get(current_geometry_gate, 0) > GEOMETRY_GATE_RANK.get(geometry_gate, 0):
            geometry_gate = current_geometry_gate

        def number(value: object, default: float = 0.0) -> float:
            try:
                return float(value)  # type: ignore[arg-type]
            except (TypeError, ValueError):
                return default

        left_rate = number(left.get("rate_hz")) if isinstance(left, dict) else 0.0
        right_rate = number(right.get("rate_hz")) if isinstance(right, dict) else 0.0
        image_rate = min(left_rate, right_rate) if left_rate and right_rate else 0.0
        image_gap = max(
            number(left.get("gap_p95_ms")) if isinstance(left, dict) else 0.0,
            number(right.get("gap_p95_ms")) if isinstance(right, dict) else 0.0,
        )
        clipping = max(
            number(left_window.get("bright_pct")) if isinstance(left_window, dict) else 0.0,
            number(right_window.get("bright_pct")) if isinstance(right_window, dict) else 0.0,
        )
        stereo_sync_samples = int(number(stereo_sync.get("samples"))) if isinstance(stereo_sync, dict) else 0
        stereo_sync_p95_us = number(stereo_sync.get("p95_us"), float("nan")) if isinstance(stereo_sync, dict) else float("nan")
        vo_ratio = self.odom_vo_valid_ratio
        if vo_ratio is None:
            state = str(self.odom_vo_state)
            vo_ratio = 100.0 if state in {"1", "1.0"} else (0.0 if state not in {"—", ""} else None)

        if not quality.get("connected") or self.odom_samples < 3 or stereo_sync_samples < 3:
            return {
                "overall": "WAIT", "label": "資料不足，等待影像與 odom",
                "score": None, "vo_valid_ratio": vo_ratio,
                "odom_rate_hz": image_rate and self.odom_rate_hz or None,
                "image_gap_p95_ms": image_gap or None,
                "stereo_sync_p95_us": stereo_sync_p95_us if stereo_sync_samples else None,
                "clipping_pct": clipping if quality.get("connected") else None,
                "path_points": len(self.odom_path), "warnings": [],
                "input_flags": input_flags,
                "current_input_flags": current_input_flags,
                "quality_gate": "WAIT",
                "geometry_gate": geometry_gate,
                "current_geometry_gate": current_geometry_gate,
                "worst_geometry": self.worst_geometry_summary(),
                "vo_ratio_source": self.odom_vo_ratio_source,
                "vo_ratio_samples": self.odom_vo_ratio_samples,
                "turn_jitter": self.turn_jitter_summary(),
                "turn_translation": self.turn_translation_summary(),
                "estimator": self.estimator_assessment(),
                "estimator_contribution": dict(self.estimator_contribution),
                "estimator_covariance": self.estimator_covariance_summary(),
                "ekf_internal": self.ekf_internal_summary(),
            }

        checks: list[tuple[str, float, float, float, int]] = []
        # value, pass threshold, review threshold, weight. Higher is better
        # for rate and VO ratio; lower is better for gap and clipping.
        checks.append(("影像 rate", image_rate, 28.0, 25.0, 10))
        checks.append(("影像 gap", image_gap, 70.0, 100.0, 10))
        checks.append(("左右同步", stereo_sync_p95_us, 100.0, 500.0, 15))
        checks.append(("clipped", clipping, 5.0, 10.0, 15))
        checks.append(("VO 有效率", float(vo_ratio or 0.0), 98.0, 95.0, 30))
        checks.append(("odom rate", self.odom_rate_hz, 25.0, 20.0, 20))

        score = 0.0
        warnings: list[str] = []
        statuses: list[str] = []
        for label, value, pass_threshold, review_threshold, weight in checks:
            higher_is_better = label in {"影像 rate", "VO 有效率", "odom rate"}
            passed = value >= pass_threshold if higher_is_better else value <= pass_threshold
            reviewed = value >= review_threshold if higher_is_better else value <= review_threshold
            if passed:
                score += weight
                statuses.append("PASS")
            elif reviewed:
                score += weight * 0.5
                statuses.append("REVIEW")
                warnings.append(f"{label}需注意")
            else:
                statuses.append("FAIL")
                warnings.append(f"{label}不合格")

        overall = "FAIL" if "FAIL" in statuses else ("REVIEW" if "REVIEW" in statuses else "PASS")
        if self.odom_vo_ratio_source != "raw /visual_slam/status samples":
            warnings.append("VO 有效率統計尚非 raw status，分數僅供參考")
        if self.turn_jitter_status == "REVIEW":
            warnings.append("轉向抖動需注意")
        if input_flags:
            warnings.append("影像品質旗標：" + ", ".join(input_flags))
            if overall == "PASS":
                overall = "REVIEW"
        if geometry_gate == "REVIEW":
            warnings.append("旋轉期間幾何需複核")
            if overall == "PASS":
                overall = "REVIEW"
        if geometry_gate == "FAIL":
            warnings.append("旋轉期間疑似出現錯誤平移")
            if self.r2_motion_label == "pure_turn":
                overall = "FAIL"
            elif overall == "PASS":
                overall = "REVIEW"
        estimator = self.estimator_assessment()
        contribution = self.estimator_contribution
        covariance = self.estimator_covariance_summary()
        estimator_gate = str(estimator.get("gate", "PENDING"))
        if estimator_gate == "REVIEW":
            warnings.append("LiDAR/EKF odom 一致性需複核")
            if overall == "PASS":
                overall = "REVIEW"
        if isinstance(contribution, dict) and contribution.get("status") == "REVIEW":
            warnings.append("EKF source 時間差過大，介入程度僅供參考")
            if overall == "PASS":
                overall = "REVIEW"
        elif estimator_gate == "FAIL":
            warnings.append("LiDAR/EKF odom 與 cuVSLAM 差異過大")
            # This is a shadow diagnostic.  It can prevent a clean PASS, but
            # it cannot declare the formal cuVSLAM TF stream unsafe by itself.
            if overall == "PASS":
                overall = "REVIEW"
        if covariance.get("warnings"):
            warnings.append("EKF source 協方差含啟用欄位 0，介入判定需複核")
            if overall == "PASS":
                overall = "REVIEW"
        label = {"PASS": "PASS：可繼續測試", "REVIEW": "REVIEW：可測但需注意", "FAIL": "FAIL：先處理輸入或追蹤"}[overall]
        return {
            "overall": overall, "label": label, "score": round(score, 1),
            "vo_valid_ratio": vo_ratio, "odom_rate_hz": self.odom_rate_hz,
            "image_gap_p95_ms": image_gap, "clipping_pct": clipping,
            "stereo_sync_p95_us": stereo_sync_p95_us,
            "path_points": len(self.odom_path), "warnings": warnings,
            "input_flags": input_flags,
            "current_input_flags": current_input_flags,
            "quality_gate": overall,
            "geometry_gate": geometry_gate,
            "current_geometry_gate": current_geometry_gate,
            "worst_geometry": self.worst_geometry_summary(),
            "vo_ratio_source": self.odom_vo_ratio_source,
            "vo_ratio_samples": self.odom_vo_ratio_samples,
            "turn_jitter": self.turn_jitter_summary(),
            "turn_translation": self.turn_translation_summary(),
            "estimator": estimator,
            "estimator_contribution": dict(contribution),
            "estimator_covariance": covariance,
            "ekf_internal": self.ekf_internal_summary(),
        }


STATE = State()


def append_run_trace(kind: str, payload: dict[str, object]) -> None:
    """Append an auditable, timestamped event without trimming the run history."""
    with STATE.lock:
        trace_file = STATE.run_trace_file
    if not trace_file:
        return
    record = {
        "schema": 1,
        "kind": kind,
        "received_at_unix": time.time(),
        "received_at_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "payload": payload,
    }
    try:
        trace_path = Path(trace_file)
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        with trace_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=True, separators=(",", ":")) + "\n")
        with STATE.lock:
            STATE.run_trace_records += 1
    except (OSError, TypeError, ValueError) as exc:
        STATE.add_log(f"完整追蹤紀錄保存失敗：{exc}")


def save_r2_session() -> str:
    """Persist the current GUI run, including the complete path and labels."""
    with STATE.lock:
        if not STATE.r2_started_at and not STATE.odom_samples:
            return ""
        QUALITY_LOG_DIR.mkdir(parents=True, exist_ok=True)
        output_path = QUALITY_LOG_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_r2_session.json"
        payload = {
            "schema": 1,
            "hardware": "Intel RealSense D435i",
            "started_at": STATE.r2_started_at,
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "mode": STATE.r2_mode,
            "mapping": STATE.r2_mapping,
            "profile": STATE.r2_profile,
            "lighting": STATE.r2_lighting,
            "diagnostics": STATE.r2_diagnostics,
            "estimator_stack": STATE.r2_estimator,
            "requested_runtime": requested_runtime_settings(
                STATE.r2_mode, STATE.r2_mapping, STATE.r2_profile,
                STATE.r2_lighting, STATE.r2_diagnostics,
                STATE.r2_scene_label, STATE.r2_motion_label, STATE.r2_estimator,
                STATE.r2_emitter_override,
            ),
            "emitter_override": STATE.r2_emitter_override,
            "scene_label": STATE.r2_scene_label,
            "motion_label": STATE.r2_motion_label,
            "summary": STATE.mapping_summary_text(),
            "position": list(STATE.odom_position),
            "yaw_deg": STATE.odom_yaw_deg,
            "run_start_yaw_deg": STATE.run_start_yaw_deg,
            "run_yaw_delta_deg": STATE.run_yaw_delta_deg,
            "run_yaw_unwrapped_deg": STATE.run_yaw_unwrapped_deg,
            "odom_path": list(STATE.odom_path),
            "odom_path_length_m": STATE.odom_path_length_m,
            "odom_samples": STATE.odom_samples,
            "turn_history": list(STATE.turn_history),
            "last_turn_translation": STATE.turn_translation_summary(),
            "worst_turn_translation": STATE.worst_geometry_summary(),
            "turn_jitter": STATE.turn_jitter_summary(),
            "image_quality": {
                **STATE.image_quality,
                "observed_flags": sorted(STATE.observed_quality_flags),
            },
            "quality_assessment": STATE.quality_assessment(),
            "estimator_sources": dict(STATE.estimator_sources),
            "estimator_comparisons": dict(STATE.estimator_comparisons),
            "estimator_contribution": dict(STATE.estimator_contribution),
            "estimator_paths": {
                name: list(points) for name, points in STATE.estimator_paths.items()
            },
            "estimator_covariance": STATE.estimator_covariance_summary(),
            "ekf_diagnostics": dict(STATE.ekf_diagnostics),
            "odom_guard_diagnostics": dict(STATE.odom_guard_diagnostics),
            "mapping_visualization": {
                "status": STATE.mapping_status,
                "map": {
                    key: value for key, value in STATE.mapping_map.items()
                    if key != "data_b64"
                },
                "scan": {
                    key: value for key, value in STATE.lidar_scan.items()
                    if key != "points"
                },
                "map_path_points": len(STATE.mapping_path),
            },
            "trace_file": STATE.run_trace_file,
            "trace_records": STATE.run_trace_records,
            "process_log": list(STATE.logs),
            "recording_name": STATE.r2_recording_name,
            "recording_mode": STATE.r2_recording_mode,
        }
        output_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=True) + "\n", encoding="utf-8")
        STATE.run_session_file = str(output_path)
        return str(output_path)


def reset_r2_live_state() -> None:
    """Clear the current experiment view without deleting saved result files."""
    with STATE.lock:
        STATE.image_quality = {"connected": False, "flags": []}
        STATE.observed_quality_flags = set()
        STATE.last_logged_metadata_signature = None
        STATE.last_logged_quality_signature = None
        STATE.last_logged_estimator_signature = None
        STATE.last_logged_contribution_signature = None
        STATE.last_logged_ekf_diagnostics_signature = None
        STATE.last_logged_odom_guard_signature = None
        STATE.last_logged_covariance_signature = None
        STATE.last_logged_scan_timing_signature = None
        STATE.estimator_sources = {}
        STATE.estimator_comparisons = {}
        STATE.estimator_contribution = {}
        STATE.ekf_diagnostics = {}
        STATE.odom_guard_diagnostics = {}
        STATE.estimator_paths = {"visual": [], "lidar": [], "ekf_shadow": []}
        STATE.estimator_path_origin = None
        STATE.estimator_path_last_samples = {}
        STATE.odom_position = [0.0, 0.0, 0.0]
        STATE.odom_yaw_deg = 0.0
        STATE.odom_rate_hz = 0.0
        STATE.odom_vo_state = "—"
        STATE.odom_vo_valid_ratio = None
        STATE.odom_vo_ratio_source = "unavailable"
        STATE.odom_vo_ratio_samples = 0
        STATE.odom_vo_valid_samples = 0
        STATE.odom_gap_p95_ms = 0.0
        STATE.odom_rate_basis = "unavailable"
        STATE.odom_timestamp_span_s = 0.0
        STATE.turn_jitter_status = "未偵測"
        STATE.turn_jitter_samples = 0
        STATE.turn_yaw_rate_p95_deg_s = 0.0
        STATE.turn_sign_flips = 0
        STATE.turn_flip_ratio = 0.0
        STATE.turn_translation_status = "未偵測"
        STATE.turn_translation_candidate_status = "未偵測"
        STATE.turn_translation_segments = 0
        STATE.turn_translation_yaw_delta_deg = 0.0
        STATE.turn_translation_max_xy_m = 0.0
        STATE.turn_translation_path_m = 0.0
        STATE.worst_geometry_gate = "未偵測"
        STATE.worst_geometry_segments = 0
        STATE.worst_geometry_yaw_delta_deg = 0.0
        STATE.worst_geometry_max_xy_m = 0.0
        STATE.worst_geometry_path_m = 0.0
        STATE.run_start_yaw_deg = 0.0
        STATE.run_yaw_delta_deg = 0.0
        STATE.run_yaw_unwrapped_deg = 0.0
        STATE.turn_history = []
        STATE.odom_telemetry_count = 0
        STATE.odom_telemetry_valid_count = 0
        STATE.telemetry_supervisor_running = False
        STATE.telemetry_reconnects = 0
        STATE.telemetry_last_data_monotonic = 0.0
        STATE.odom_path_length_offset_m = 0.0
        STATE.odom_samples_offset = 0
        STATE.odom_vo_status_offset = 0
        STATE.odom_vo_valid_offset = 0
        STATE.remote_odom_samples = 0
        STATE.remote_path_length_m = 0.0
        STATE.remote_vo_status_samples = 0
        STATE.remote_vo_valid_samples = 0
        STATE.odom_path = []
        STATE.odom_path_length_m = 0.0
        STATE.odom_samples = 0
        STATE.odom_gap_ms = 0.0
        STATE.mapping_map = {}
        STATE.mapping_status = "尚未收到 /map"
        STATE.mapping_path = []
        STATE.mapping_robot_pose = []
        STATE.lidar_scan = {
            "status": "尚未收到 /scan",
            "frame_id": "",
            "points": [],
            "source_points": 0,
            "display_points": 0,
            "alignment": {"status": "WAIT"},
            "timing": {"status": "WAIT"},
        }
        STATE.motion_recording = False
        STATE.motion_status = "尚未記錄"
        STATE.motion_started_at = 0.0
        STATE.motion_start_position = None
        STATE.motion_start_path_length_m = None
        STATE.motion_start_samples = 0
        STATE.motion_duration_s = 0.0
        STATE.motion_displacement_m = 0.0
        STATE.motion_path_length_m = 0.0
        STATE.motion_max_displacement_m = 0.0
        STATE.motion_samples = 0
        STATE.motion_quality_samples = []
        STATE.motion_record_file = ""


def ssh_command(args: list[str]) -> list[str]:
    return [
        "ssh", "-p", "22", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
        "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3",
        "-o", "ControlMaster=no", "-o", "ControlPath=none",
        "-o", "StrictHostKeyChecking=accept-new", f"{USER}@{HOST}", shlex.join(args),
    ]


def remote_script_command(candidates: tuple[str, ...], args: list[str]) -> list[str]:
    """Run a remote script from the new layout, with a legacy fallback."""
    candidate_words = " ".join(shlex.quote(path) for path in candidates)
    argument_words = shlex.join(args)
    resolver = (
        f"for script in {candidate_words}; do "
        f"if [ -f \"$script\" ]; then echo \"using remote cuVSLAM script: $script\" >&2; "
        f"exec bash \"$script\" {argument_words}; fi; "
        "done; "
        "echo 'remote cuVSLAM script not found in new or legacy workspace layout' >&2; "
        "exit 127"
    )
    return ssh_command(["bash", "-lc", resolver])


def remote_r1p5_command(*args: str) -> list[str]:
    return remote_script_command(REMOTE_SCRIPT_CANDIDATES, list(args))


def remote_r2_command(*args: str) -> list[str]:
    return remote_script_command(REMOTE_R2_SCRIPT_CANDIDATES, list(args))


def remote_rplidar_command(*args: str) -> list[str]:
    return remote_script_command(REMOTE_RPLIDAR_SCRIPT_CANDIDATES, list(args))


def remote_localization_command(*args: str) -> list[str]:
    return remote_script_command(REMOTE_LOCALIZATION_SCRIPT_CANDIDATES, list(args))


def active_runtime_command(*args: str) -> list[str]:
    with STATE.lock:
        mapping = STATE.r2_mapping
    return remote_rplidar_command(*args) if mapping == "mapping" else remote_r2_command(*args)


def active_runtime_data_dir() -> str:
    with STATE.lock:
        mapping = STATE.r2_mapping
    return REMOTE_RPLIDAR_DATA_DIR if mapping == "mapping" else REMOTE_R2_DATA_DIR


def remote_rviz_command(*args: str) -> list[str]:
    return remote_script_command(REMOTE_RVIZ_VIEW_SCRIPT_CANDIDATES, list(args))


def ssh_tunnel_command() -> list[str]:
    return [
        "ssh", "-N", "-L", "5901:127.0.0.1:5900", "-p", "22",
        "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
        "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3",
        "-o", "ExitOnForwardFailure=yes", "-o", "ControlMaster=no",
        "-o", "ControlPath=none", "-o", "StrictHostKeyChecking=accept-new",
        f"{USER}@{HOST}",
    ]


def local_vnc_port_open() -> bool:
    try:
        with socket.create_connection(("127.0.0.1", 5901), timeout=0.4):
            return True
    except OSError:
        return False


def ensure_rviz_tunnel() -> None:
    with STATE.lock:
        process = STATE.rviz_tunnel_process
        if process is not None and process.poll() is None:
            return
    if local_vnc_port_open():
        STATE.add_log("沿用既有 SSH VNC tunnel：127.0.0.1:5901")
        return
    process = subprocess.Popen(
        ssh_tunnel_command(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    with STATE.lock:
        STATE.rviz_tunnel_process = process
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        if local_vnc_port_open():
            STATE.add_log("SSH VNC tunnel 已建立：127.0.0.1:5901 → Jetson:5900")
            return
        if process.poll() is not None:
            error = process.stderr.read().strip() if process.stderr else ""
            raise RuntimeError(error or "SSH VNC tunnel 建立失敗")
        time.sleep(0.25)
    process.terminate()
    raise RuntimeError("等待 SSH VNC tunnel 超時")


def open_mac_screen_sharing() -> None:
    """Open the native macOS VNC client and fill the first-login dialog."""
    subprocess.run(
        ["open", "vnc://127.0.0.1:5901"],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    # Screen Sharing remembers the password after the first successful login.
    # When a local password is configured, UI scripting fills the localized
    # authentication sheet. Otherwise the normal macOS login dialog remains
    # available for manual entry.
    if not VNC_PASSWORD:
        return
    escaped_password = VNC_PASSWORD.replace("\\", "\\\\").replace('"', '\\"')
    script = f'''tell application "System Events"
  repeat 30 times
    if exists process "Screen Sharing" then
      tell process "Screen Sharing"
        if exists window 1 then
          try
            if exists sheet 1 of window 1 then
              set authSheet to sheet 1 of window 1
              if exists text field 1 of authSheet then set value of text field 1 of authSheet to "{escaped_password}"
              if exists checkbox 1 of authSheet then
                if value of checkbox 1 of authSheet is 0 then click checkbox 1 of authSheet
              end if
              if exists button "登入" of authSheet then
                click button "登入" of authSheet
              else if exists button "Log In" of authSheet then
                click button "Log In" of authSheet
              else if exists button "Connect" of authSheet then
                click button "Connect" of authSheet
              end if
            end if
          end try
          set frontmost to true
          exit repeat
        end if
      end tell
    end if
    delay 0.2
  end repeat
end tell'''
    subprocess.run(
        ["osascript", "-e", script],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        timeout=12,
    )


def rviz_open_worker() -> None:
    try:
        result = subprocess.run(
            remote_rviz_command("open"),
            capture_output=True,
            text=True,
            timeout=90,
        )
        output = (result.stdout + result.stderr).strip()
        if output:
            for line in output.splitlines()[-30:]:
                STATE.add_log(line)
        if result.returncode != 0:
            raise RuntimeError(output or "Jetson RViz viewer 啟動失敗")
        ensure_rviz_tunnel()
        open_mac_screen_sharing()
        message = "RViz 已開啟於 macOS「螢幕共享」；可直接點擊、拖曳與縮放"
        with STATE.lock:
            STATE.rviz_status = message
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(f"RViz 視窗開啟失敗：{exc}")
        with STATE.lock:
            STATE.rviz_status = f"RViz 開啟失敗：{exc}"
    finally:
        with STATE.lock:
            STATE.rviz_opening = False


def open_rviz() -> str:
    with STATE.lock:
        if STATE.rviz_opening:
            return "正在開啟 RViz 視窗…"
        STATE.rviz_opening = True
        STATE.rviz_status = "正在啟動 Jetson RViz、x11vnc 與 SSH tunnel…"
    threading.Thread(target=rviz_open_worker, daemon=True).start()
    return "正在開啟 RViz 可操控視窗…"


def handle_map_telemetry_line(line: str) -> None:
    try:
        data = json.loads(line[len("LIVE_MAP "):])
        map_data = data.get("map", {})
        if not isinstance(map_data, dict):
            raise ValueError("map payload is not an object")
        width = int(map_data.get("width", 0))
        height = int(map_data.get("height", 0))
        resolution = float(map_data.get("resolution", 0.0))
        encoded = str(map_data.get("data_b64", ""))
        if not 1 <= width <= 4096 or not 1 <= height <= 4096:
            raise ValueError(f"invalid map size {width}x{height}")
        if resolution <= 0.0 or not encoded:
            raise ValueError("map resolution or occupancy data is missing")
        if len(encoded) > 16_000_000:
            raise ValueError("map occupancy payload is too large")
        stored_map = dict(map_data)
        with STATE.lock:
            STATE.mapping_map = stored_map
            frame = str(stored_map.get("frame_id", "map"))
            revision = int(stored_map.get("revision", 0))
            STATE.mapping_status = f"/map {width}×{height} · {frame} · rev {revision}"
        trace_map = {key: value for key, value in stored_map.items() if key != "data_b64"}
        append_run_trace("map_telemetry", {"schema": data.get("schema", 1), "map": trace_map})
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        STATE.add_log(f"map telemetry parse failed: {exc}")


def handle_scan_telemetry_line(line: str) -> None:
    try:
        data = json.loads(line[len("LIVE_SCAN "):])
        raw_points = data.get("points", [])
        points: list[list[float]] = []
        if isinstance(raw_points, list):
            for point in raw_points:
                if not isinstance(point, (list, tuple)) or len(point) < 2:
                    continue
                try:
                    x, y = float(point[0]), float(point[1])
                except (TypeError, ValueError):
                    continue
                if math.isfinite(x) and math.isfinite(y):
                    points.append([x, y])
        stored_scan = dict(data)
        stored_scan["points"] = points
        stored_scan["source_points"] = int(data.get("source_points", 0))
        stored_scan["display_points"] = len(points)
        with STATE.lock:
            STATE.lidar_scan = stored_scan
            timing = stored_scan.get("timing", {})
            if isinstance(timing, dict):
                timing_signature = (
                    timing.get("status"),
                    timing.get("scan_stamp_ns"),
                    timing.get("scan_time_s"),
                    timing.get("time_increment_s"),
                    timing.get("beam_span_ms"),
                    timing.get("timestamp_age_ms"),
                    timing.get("scan_end_age_ms"),
                    timing.get("tf_lookup_status"),
                    timing.get("tf_lookup_delta_ms"),
                    timing.get("tf_latest_delta_ms"),
                    timing.get("display_mode"),
                    timing.get("display_delay_ms"),
                    timing.get("queue_depth"),
                    timing.get("queue_drops"),
                    timing.get("tf_error"),
                )
                if timing_signature != STATE.last_logged_scan_timing_signature:
                    STATE.last_logged_scan_timing_signature = timing_signature
                    STATE.add_log("=== LiDAR 時間診斷 ===")
                    STATE.add_log(
                        f"scan_stamp={timing.get('scan_stamp_ns', '—')} · "
                        f"scan_time={timing.get('scan_time_s', '—')} s · "
                        f"beam_span={timing.get('beam_span_ms', '—')} ms · "
                        f"time_increment={timing.get('time_increment_us', '—')} µs · "
                        f"scan_age={timing.get('timestamp_age_ms', '—')} ms · "
                        f"scan_end_age={timing.get('scan_end_age_ms', '—')} ms"
                    )
                    STATE.add_log(
                        f"TF lookup={timing.get('tf_lookup_status', '—')} · "
                        f"scan↔TF Δ={timing.get('tf_lookup_delta_ms', '—')} ms · "
                        f"latest TF Δ={timing.get('tf_latest_delta_ms', '—')} ms · "
                        f"display={timing.get('display_mode', '—')} · "
                        f"delay={timing.get('display_delay_ms', '—')} ms · "
                        f"queue={timing.get('queue_depth', '—')} · "
                        f"drops={timing.get('queue_drops', '—')} · "
                        f"error={timing.get('tf_error') or 'none'}"
                    )
        trace_scan = dict(stored_scan)
        trace_scan.pop("points", None)
        append_run_trace("scan_telemetry", trace_scan)
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        STATE.add_log(f"scan telemetry parse failed: {exc}")


def handle_telemetry_line(line: str) -> None:
    if line.startswith("LIVE_MAP "):
        handle_map_telemetry_line(line)
        return
    if line.startswith("LIVE_SCAN "):
        handle_scan_telemetry_line(line)
        return
    if not line.startswith("LIVE_ODOM "):
        if line:
            STATE.add_log(line)
        return
    try:
        data = json.loads(line[len("LIVE_ODOM "):])
        with STATE.lock:
            incoming_localization = data.get("localization_pose")
            if isinstance(incoming_localization, dict):
                STATE.localization_pose = {
                    "available": bool(incoming_localization.get("available", False)),
                    "status": str(incoming_localization.get("status", "WAIT_AMCL")),
                    "frame_id": str(incoming_localization.get("frame_id", "")),
                    "x_m": incoming_localization.get("x_m"),
                    "y_m": incoming_localization.get("y_m"),
                    "yaw_deg": incoming_localization.get("yaw_deg"),
                    "covariance_x_m2": incoming_localization.get("covariance_x_m2"),
                    "covariance_y_m2": incoming_localization.get("covariance_y_m2"),
                    "covariance_yaw_rad2": incoming_localization.get("covariance_yaw_rad2"),
                }
            remote_samples = int(data.get("samples", 0))
            remote_path_length_m = float(data.get("path_length_m", 0.0))
            remote_status_samples = int(data.get("vo_status_samples", 0))
            remote_valid_samples = int(data.get("vo_valid_samples", 0))
            remote_restarted = (
                STATE.remote_odom_samples > 0
                and remote_samples < STATE.remote_odom_samples
            ) or (
                STATE.remote_path_length_m > 0.0
                and remote_path_length_m + 1e-9 < STATE.remote_path_length_m
            )
            if remote_restarted:
                # The remote collector has its own counters.  A reconnect
                # starts those counters again, so carry their previous totals
                # into the local session before accepting the new snapshot.
                STATE.odom_samples_offset += STATE.remote_odom_samples
                STATE.odom_path_length_offset_m += STATE.remote_path_length_m
                STATE.odom_vo_status_offset += STATE.remote_vo_status_samples
                STATE.odom_vo_valid_offset += STATE.remote_vo_valid_samples
                STATE.add_log(
                    "odom telemetry 已恢復；保留既有 session 累積資料，"
                    f"自動重連次數={STATE.telemetry_reconnects}"
                )
                STATE.estimator_paths = {"visual": [], "lidar": [], "ekf_shadow": []}
                STATE.estimator_path_origin = None
                STATE.estimator_path_last_samples = {}
            STATE.remote_odom_samples = remote_samples
            STATE.remote_path_length_m = remote_path_length_m
            STATE.remote_vo_status_samples = remote_status_samples
            STATE.remote_vo_valid_samples = remote_valid_samples
            STATE.telemetry_last_data_monotonic = time.monotonic()
            STATE.odom_position = [float(value) for value in data.get("position", [0, 0, 0])]
            STATE.odom_yaw_deg = float(data.get("yaw_deg", 0.0))
            STATE.odom_rate_hz = float(data.get("odom_rate_hz", 0.0))
            STATE.odom_vo_state = data.get("vo_state", "—")
            STATE.odom_gap_p95_ms = float(data.get("gap_p95_ms", data.get("last_gap_ms", 0.0)))
            STATE.odom_rate_basis = str(data.get("odom_rate_basis", "collector wall-clock elapsed"))
            STATE.odom_timestamp_span_s = float(data.get("odom_timestamp_span_s", 0.0))
            run_start_yaw = data.get("run_start_yaw_deg")
            if run_start_yaw is not None:
                STATE.run_start_yaw_deg = float(run_start_yaw)
            STATE.run_yaw_delta_deg = float(data.get("run_yaw_delta_deg", 0.0))
            STATE.run_yaw_unwrapped_deg = float(data.get("run_yaw_unwrapped_deg", 0.0))
            history = data.get("turn_history", [])
            if isinstance(history, list):
                STATE.turn_history = [item for item in history if isinstance(item, dict)]
                for item in STATE.turn_history:
                    STATE.observe_geometry(
                        item.get("candidate_status", "未偵測"),
                        item.get("turn_segments", 0),
                        item.get("yaw_delta_deg", 0.0),
                        item.get("max_xy_displacement_m", 0.0),
                        item.get("path_length_m", 0.0),
                    )
            jitter = data.get("turn_jitter", {})
            if isinstance(jitter, dict):
                previous_jitter_status = STATE.turn_jitter_status
                STATE.turn_jitter_status = str(jitter.get("status", "未偵測"))
                STATE.turn_jitter_samples = int(jitter.get("turn_samples", 0))
                STATE.turn_yaw_rate_p95_deg_s = float(jitter.get("yaw_rate_p95_deg_s", 0.0))
                STATE.turn_sign_flips = int(jitter.get("sign_flips", 0))
                STATE.turn_flip_ratio = float(jitter.get("flip_ratio", 0.0))
                if STATE.turn_jitter_status == "REVIEW" and previous_jitter_status != "REVIEW":
                    STATE.add_log(
                        "轉向抖動診斷進入 REVIEW："
                        f"sign_flips={STATE.turn_sign_flips} "
                        f"yaw_rate_p95={STATE.turn_yaw_rate_p95_deg_s:.1f} deg/s"
                    )
            turn_translation = data.get("turn_translation", {})
            if isinstance(turn_translation, dict):
                previous_translation_status = STATE.turn_translation_status
                STATE.turn_translation_status = str(turn_translation.get("status", "未偵測"))
                STATE.turn_translation_candidate_status = str(turn_translation.get("candidate_status", "未偵測"))
                STATE.turn_translation_segments = int(turn_translation.get("turn_segments", 0))
                STATE.turn_translation_yaw_delta_deg = float(turn_translation.get("yaw_delta_deg", 0.0))
                STATE.turn_translation_max_xy_m = float(turn_translation.get("max_xy_displacement_m", 0.0))
                STATE.turn_translation_path_m = float(turn_translation.get("path_length_m", 0.0))
                STATE.observe_geometry(
                    STATE.turn_translation_candidate_status,
                    STATE.turn_translation_segments,
                    STATE.turn_translation_yaw_delta_deg,
                    STATE.turn_translation_max_xy_m,
                    STATE.turn_translation_path_m,
                )
                if STATE.turn_translation_candidate_status in {"REVIEW", "FAIL"} and previous_translation_status != STATE.turn_translation_status:
                    STATE.add_log(
                        f"Geometry Gate: {STATE.turn_translation_candidate_status}；"
                        "旋轉期間估計平移候選進入 "
                        f"{STATE.turn_translation_candidate_status}（待確認動作類型）："
                        f"yaw_delta={STATE.turn_translation_yaw_delta_deg:.1f} deg "
                        f"max_xy={STATE.turn_translation_max_xy_m:.3f} m"
                    )
            incoming_sources = data.get("estimator_sources", {})
            incoming_comparisons = data.get("estimator_comparisons", {})
            incoming_contribution = data.get("estimator_contribution", {})
            incoming_ekf_diagnostics = data.get("ekf_diagnostics", {})
            if isinstance(incoming_sources, dict):
                STATE.estimator_sources = {
                    str(name): dict(source)
                    for name, source in incoming_sources.items()
                    if isinstance(source, dict)
                }
                STATE.update_estimator_paths(STATE.estimator_sources)
            if isinstance(incoming_comparisons, dict):
                STATE.estimator_comparisons = {
                    str(name): dict(comparison)
                    for name, comparison in incoming_comparisons.items()
                    if isinstance(comparison, dict)
                }
            if isinstance(incoming_contribution, dict):
                STATE.estimator_contribution = dict(incoming_contribution)
            if isinstance(incoming_ekf_diagnostics, dict):
                STATE.ekf_diagnostics = dict(incoming_ekf_diagnostics)
            incoming_odom_guard_diagnostics = data.get("odom_guard_diagnostics", {})
            if isinstance(incoming_odom_guard_diagnostics, dict):
                STATE.odom_guard_diagnostics = dict(incoming_odom_guard_diagnostics)
            guard_sources = STATE.odom_guard_diagnostics.get("sources", {})
            if isinstance(guard_sources, dict):
                guard_signature = tuple(
                    (
                        str(source),
                        details.get("level"),
                        details.get("message"),
                        (
                            details.get("values", {}).get("accepted")
                            if isinstance(details.get("values", {}), dict)
                            else None
                        ),
                    )
                    for source, details in sorted(guard_sources.items())
                    if isinstance(details, dict)
                )
                if guard_signature != STATE.last_logged_odom_guard_signature:
                    STATE.last_logged_odom_guard_signature = guard_signature
                    for source, details in sorted(guard_sources.items()):
                        if not isinstance(details, dict):
                            continue
                        values = details.get("values", {})
                        accepted = (
                            values.get("accepted", "unknown")
                            if isinstance(values, dict)
                            else "unknown"
                        )
                        STATE.add_log(
                            "Odometry input guard："
                            f"{source} accepted={accepted} "
                            f"status={details.get('message', '—')}"
                        )
            if STATE.estimator_sources:
                source_status = tuple(
                    f"{name}:{source.get('status', 'WAIT')}"
                    for name, source in sorted(STATE.estimator_sources.items())
                )
                comparison_status = tuple(
                    f"{name}:{comparison.get('gate', 'PENDING')}"
                    for name, comparison in sorted(STATE.estimator_comparisons.items())
                )
                estimator_signature = source_status + comparison_status
                if estimator_signature != STATE.last_logged_estimator_signature:
                    STATE.last_logged_estimator_signature = estimator_signature
                    lidar_source = STATE.estimator_sources.get("lidar", {})
                    ekf_source = STATE.estimator_sources.get("ekf_shadow", {})
                    visual_lidar = STATE.estimator_comparisons.get("visual_vs_lidar", {})
                    STATE.add_log(
                        "Estimator shadow："
                        f"lidar={lidar_source.get('status', 'WAIT')} "
                        f"ekf={ekf_source.get('status', 'WAIT')} "
                        f"visual_vs_lidar={visual_lidar.get('gate', 'PENDING')} "
                        f"skew={float(visual_lidar.get('timestamp_skew_ms', 0.0)):.1f} ms"
                    )
                    for comparison_name, comparison in sorted(STATE.estimator_comparisons.items()):
                        if isinstance(comparison, dict) and "timestamp_skew_ms" in comparison:
                            STATE.add_log(
                                f"{comparison_name} timestamp skew="
                                f"{float(comparison.get('timestamp_skew_ms', 0.0)):.1f} ms"
                            )
            covariance_summary = STATE.estimator_covariance_summary()
            covariance_signature = tuple(
                (
                    name,
                    details.get("pose"),
                    details.get("twist"),
                    tuple(details.get("pose_planar_zero_indices", [])),
                    tuple(details.get("twist_planar_zero_indices", [])),
                )
                for name, details in sorted(covariance_summary.get("sources", {}).items())
                if isinstance(details, dict)
            )
            if covariance_signature != STATE.last_logged_covariance_signature:
                STATE.last_logged_covariance_signature = covariance_signature
                if covariance_summary.get("warnings"):
                    STATE.add_log(
                        "協方差警告（只標示，不改寫 source）："
                        + " · ".join(str(item) for item in covariance_summary["warnings"])
                    )
                elif covariance_signature:
                    STATE.add_log("協方差觀察：目前未見啟用平面欄位為零")
            contribution = STATE.estimator_contribution
            pull_estimate = contribution.get("pull_estimate", {}) if isinstance(contribution, dict) else {}
            pull_xy = pull_estimate.get("xy", {}) if isinstance(pull_estimate, dict) else {}
            pull_yaw = pull_estimate.get("yaw", {}) if isinstance(pull_estimate, dict) else {}
            contribution_signature = (
                contribution.get("status", "WAIT") if isinstance(contribution, dict) else "WAIT",
                pull_xy.get("lidar_pct") if isinstance(pull_xy, dict) else None,
                pull_yaw.get("lidar_pct") if isinstance(pull_yaw, dict) else None,
            )
            if contribution_signature != STATE.last_logged_contribution_signature:
                STATE.last_logged_contribution_signature = contribution_signature
                if isinstance(contribution, dict) and contribution.get("status") in {"OK", "REVIEW"}:
                    STATE.add_log(
                        "EKF 介入程度（軌跡推估，非 Kalman gain）："
                        f"XY LiDAR={pull_xy.get('lidar_pct', '—')}% "
                        f"cuVSLAM={pull_xy.get('cuvslam_pct', '—')}% · "
                        f"yaw LiDAR={pull_yaw.get('lidar_pct', '—')}% "
                        f"cuVSLAM={pull_yaw.get('cuvslam_pct', '—')}%"
                    )
            internal = STATE.ekf_internal_summary()
            internal_signature = (
                internal.get("status"), internal.get("processed"),
                internal.get("rejected"), internal.get("dropped"),
                internal.get("frequency"),
            )
            if internal_signature != STATE.last_logged_ekf_diagnostics_signature:
                STATE.last_logged_ekf_diagnostics_signature = internal_signature
                if internal.get("available"):
                    STATE.add_log("=== EKF 內部診斷 ===")
                    STATE.add_log(
                        f"Status: {internal.get('status', 'UNKNOWN')} · "
                        f"processed={internal.get('processed') or '—'} · "
                        f"rejected={internal.get('rejected') or '—'} · "
                        f"dropped={internal.get('dropped') or '—'} · "
                        f"frequency={internal.get('frequency') or '—'}"
                    )
                    if internal.get("message"):
                        STATE.add_log(f"Message: {internal.get('message')}")
            incoming_map_path = data.get("map_path")
            if isinstance(incoming_map_path, list):
                STATE.mapping_path = [
                    [float(point[0]), float(point[1])]
                    for point in incoming_map_path
                    if isinstance(point, (list, tuple)) and len(point) >= 2
                ]
            incoming_robot_pose = data.get("robot_pose_map")
            if isinstance(incoming_robot_pose, list) and len(incoming_robot_pose) >= 2:
                try:
                    STATE.mapping_robot_pose = [float(value) for value in incoming_robot_pose[:3]]
                except (TypeError, ValueError):
                    STATE.mapping_robot_pose = []
            if data.get("vo_valid_ratio") is not None:
                total_status_samples = STATE.odom_vo_status_offset + remote_status_samples
                total_valid_samples = STATE.odom_vo_valid_offset + remote_valid_samples
                STATE.odom_vo_valid_ratio = (
                    100.0 * total_valid_samples / total_status_samples
                    if total_status_samples else None
                )
                STATE.odom_vo_ratio_source = "raw /visual_slam/status samples"
                STATE.odom_vo_ratio_samples = total_status_samples
                STATE.odom_vo_valid_samples = total_valid_samples
            else:
                # Backward-compatible fallback while an older Jetson
                # collector is still running.  This is deliberately marked
                # provisional: a periodic telemetry snapshot is not a raw
                # status-message sample and must not be reported as formal
                # VO validity.
                STATE.odom_telemetry_count += 1
                if str(STATE.odom_vo_state) in {"1", "1.0"}:
                    STATE.odom_telemetry_valid_count += 1
                STATE.odom_vo_valid_ratio = (
                    100.0 * STATE.odom_telemetry_valid_count / STATE.odom_telemetry_count
                )
                STATE.odom_vo_ratio_source = "GUI telemetry fallback"
                STATE.odom_vo_ratio_samples = STATE.odom_telemetry_count
                STATE.odom_vo_valid_samples = STATE.odom_telemetry_valid_count
            # Keep the complete trajectory for this R2 run.  Older Jetson
            # collectors may still send a sliding window, so merge the new
            # window after the last point already held by the GUI instead of
            # replacing the complete local history with that window.
            incoming_path = data.get("path", [])
            if incoming_path:
                incoming_path = [list(point) for point in incoming_path]
                if not STATE.odom_path:
                    STATE.odom_path = incoming_path
                else:
                    last_point = STATE.odom_path[-1]
                    overlap = next(
                        (index for index in range(len(incoming_path) - 1, -1, -1)
                         if incoming_path[index] == last_point),
                        None,
                    )
                    if overlap is None:
                        # This can happen if the telemetry process was
                        # restarted independently.  Preserve both segments;
                        # a new R2 start clears the state explicitly.
                        STATE.odom_path.extend(incoming_path)
                    else:
                        STATE.odom_path.extend(incoming_path[overlap + 1:])
            STATE.odom_path_length_m = STATE.odom_path_length_offset_m + remote_path_length_m
            STATE.odom_samples = STATE.odom_samples_offset + remote_samples
            STATE.odom_gap_ms = float(data.get("last_gap_ms", 0.0))
            if STATE.motion_recording:
                if STATE.motion_start_position is None:
                    STATE.motion_start_position = list(STATE.odom_position)
                    STATE.motion_start_path_length_m = STATE.odom_path_length_m
                    STATE.motion_start_samples = STATE.odom_samples
                start = STATE.motion_start_position
                STATE.motion_duration_s = time.monotonic() - STATE.motion_started_at
                STATE.motion_displacement_m = math.sqrt(sum((a - b) ** 2 for a, b in zip(start, STATE.odom_position)))
                baseline_path = (STATE.motion_start_path_length_m
                                 if STATE.motion_start_path_length_m is not None
                                 else STATE.odom_path_length_m)
                STATE.motion_path_length_m = max(0.0, STATE.odom_path_length_m - baseline_path)
                STATE.motion_max_displacement_m = max(STATE.motion_max_displacement_m, STATE.motion_displacement_m)
                STATE.motion_samples = max(0, STATE.odom_samples - STATE.motion_start_samples)
            if STATE.localization_running:
                STATE.localization_status = "定位 telemetry 連線中"
            else:
                STATE.r2_status = (
                    "2D SLAM 即時連線中"
                    if STATE.r2_mapping == "mapping" else "R2 odom 即時連線中"
                )
        trace_data = dict(data)
        trace_data.pop("path", None)
        trace_data.pop("map_path", None)
        trace_data["path_point_count"] = len(data.get("path", [])) if isinstance(data.get("path", []), list) else 0
        trace_data["map_path_point_count"] = len(data.get("map_path", [])) if isinstance(data.get("map_path", []), list) else 0
        append_run_trace("odom_telemetry", trace_data)
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        STATE.add_log(f"telemetry parse failed: {exc}")


def telemetry_reader(process: subprocess.Popen[str]) -> None:
    assert process.stdout is not None
    for raw_line in process.stdout:
        handle_telemetry_line(raw_line.strip())


def telemetry_supervisor() -> None:
    """Keep the active R2, 2D-SLAM, or localization telemetry stream alive."""
    first_connection = True
    while True:
        with STATE.lock:
            active = STATE.r2_running or STATE.localization_running
            telemetry_mode = STATE.telemetry_mode
            if not active:
                break
            STATE.telemetry_last_data_monotonic = time.monotonic()
            if telemetry_mode == "localization":
                runtime_label = "定位"
                telemetry_command = remote_localization_command("telemetry")
            else:
                runtime_label = "2D SLAM" if STATE.r2_mapping == "mapping" else "R2 odom"
                telemetry_command = active_runtime_command("telemetry")
        try:
            process = subprocess.Popen(
                telemetry_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            with STATE.lock:
                if not (STATE.r2_running or STATE.localization_running):
                    break
                STATE.telemetry_reconnects += 1
                if STATE.localization_running:
                    STATE.localization_status = f"{runtime_label} telemetry 連線失敗，正在自動重連…"
                else:
                    STATE.r2_status = f"{runtime_label} telemetry 連線失敗，正在自動重連…"
                STATE.add_log(f"{runtime_label} telemetry SSH 啟動失敗：{exc}")
            time.sleep(1.0)
            first_connection = False
            continue

        with STATE.lock:
            STATE.telemetry_process = process
            if not first_connection:
                STATE.add_log(f"{runtime_label} telemetry SSH 已重新建立，等待 LIVE_ODOM")
            if STATE.localization_running:
                STATE.localization_status = f"{runtime_label} 即時連線中"
            else:
                STATE.r2_status = f"{runtime_label} 即時連線中"

        reader = threading.Thread(target=telemetry_reader, args=(process,), daemon=True)
        reader.start()
        while reader.is_alive():
            with STATE.lock:
                running = STATE.r2_running or STATE.localization_running
                last_data = STATE.telemetry_last_data_monotonic
            if not running:
                if process.poll() is None:
                    process.terminate()
                break
            if process.poll() is not None:
                break
            # The collector normally emits LIVE_ODOM every 200 ms.  A long
            # silent period means the SSH stream or remote ROS collector is
            # stuck even if the ssh process itself still exists.
            if last_data and time.monotonic() - last_data > 20.0:
                with STATE.lock:
                    STATE.add_log(f"{runtime_label} telemetry 超過 20 秒沒有資料，重新建立連線")
                process.terminate()
                break
            time.sleep(0.5)
        reader.join(timeout=2.0)
        if process.poll() is None:
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        with STATE.lock:
            if STATE.telemetry_process is process:
                STATE.telemetry_process = None
            running = STATE.r2_running or STATE.localization_running
        if not running:
            break
        with STATE.lock:
            STATE.telemetry_reconnects += 1
            if STATE.localization_running:
                STATE.localization_status = f"{runtime_label} 已啟動，但 telemetry 中斷，正在自動重連…"
            else:
                STATE.r2_status = f"{runtime_label} 已啟動，但 telemetry 中斷，正在自動重連…"
            STATE.add_log(
                f"{runtime_label} telemetry 已中斷，將自動重連；"
                f"第 {STATE.telemetry_reconnects} 次"
            )
        time.sleep(1.0)
        first_connection = False

    with STATE.lock:
        STATE.telemetry_supervisor_running = False
        if STATE.telemetry_process is not None and STATE.telemetry_process.poll() is not None:
            STATE.telemetry_process = None


def metadata_number(metadata: object, field: str) -> float | None:
    if not isinstance(metadata, dict):
        return None
    for key in ("latest", "window"):
        values = metadata.get(key, {})
        if not isinstance(values, dict) or field not in values:
            continue
        try:
            value = float(values[field])
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            return value
    return None


def metadata_pair(metadata: object, field: str, decimals: int = 0) -> str:
    values = metadata if isinstance(metadata, dict) else {}
    left = metadata_number(values.get("left", {}), field)
    right = metadata_number(values.get("right", {}), field)
    if left is None and right is None:
        return "—"
    formatter = f"{{:.{decimals}f}}"
    return f"{formatter.format(left) if left is not None else '—'} / {formatter.format(right) if right is not None else '—'}"


def metadata_bucket_pair(metadata: object, field: str, bucket: float) -> tuple[int | None, int | None]:
    values = metadata if isinstance(metadata, dict) else {}
    result: list[int | None] = []
    for side in ("left", "right"):
        value = metadata_number(values.get(side, {}), field)
        result.append(None if value is None else int(round(value / bucket)))
    return result[0], result[1]


def metadata_switch_pair(metadata: object, field: str) -> str:
    values = metadata if isinstance(metadata, dict) else {}
    result: list[str] = []
    for side in ("left", "right"):
        value = metadata_number(values.get(side, {}), field)
        result.append("—" if value is None else ("ON" if value != 0.0 else "OFF"))
    return " / ".join(result)


def log_requested_runtime(
    mode: str, mapping: str, profile: str, lighting: str,
    diagnostics: str, scene: str, motion: str, estimator_stack: str = "cuvslam",
    emitter_override: str = "profile",
) -> None:
    settings = requested_runtime_settings(
        mode, mapping, profile, lighting, diagnostics, scene, motion,
        estimator_stack, emitter_override,
    )
    d435i = settings["d435i_requested"]
    cuvslam = settings["cuvslam_requested"]
    assert isinstance(d435i, dict)
    assert isinstance(cuvslam, dict)
    STATE.add_log("=== Runtime 實驗設定（Requested） ===")
    STATE.add_log(f"Mode: {mode}")
    STATE.add_log(f"Mapping: {mapping}")
    STATE.add_log(f"Profile: {profile}")
    STATE.add_log(f"Lighting: {lighting}")
    STATE.add_log(f"Diagnostics: {diagnostics}")
    STATE.add_log(f"Estimator Stack: {estimator_stack}")
    STATE.add_log(f"IR Projector A/B Override: {emitter_override.upper()}")
    STATE.add_log(
        f"LiDAR fixed mount TF: base_link -> rplidar_link, "
        f"x={LIDAR_X} m y={LIDAR_Y} m z={LIDAR_Z} m yaw={LIDAR_YAW} rad "
        "(software override; physical mount unchanged)"
    )
    STATE.add_log(f"Scene Label: {scene}")
    STATE.add_log(f"Motion Label: {motion}")
    STATE.add_log("=== D435i 設定（Requested） ===")
    STATE.add_log(f"IR Projector: {d435i.get('ir_projector', '—')}")
    STATE.add_log(f"Exposure Mode: {'Manual' if d435i.get('auto_exposure') == 'OFF' else 'Automatic'}")
    STATE.add_log(f"Requested Exposure: {d435i.get('exposure_us', '—')} us")
    STATE.add_log(f"Requested Gain: {d435i.get('gain', '—')}")
    STATE.add_log("=== cuVSLAM Diagnostics（Requested） ===")
    STATE.add_log(
        f"Observations View / Recording: {'ON' if cuvslam.get('observations_view') else 'OFF'}"
    )
    STATE.add_log(
        "Landmarks / Pose Graph / Loop Closure: full recording mode available; "
        "not recording yet"
    )
    STATE.add_log(f"Denoise Input Images: {d435i.get('denoise', '—')}")


def handle_quality_line(line: str) -> None:
    if not line.startswith("LIVE_QUALITY "):
        if line:
            STATE.add_log(line)
        return
    try:
        data = json.loads(line[len("LIVE_QUALITY "):])
        with STATE.lock:
            data["connected"] = True
            flags = tuple(str(flag) for flag in data.get("flags", []))
            STATE.observed_quality_flags.update(flags)
            data["observed_flags"] = sorted(STATE.observed_quality_flags)
            STATE.image_quality = data
            metadata = data.get("metadata", {})
            actual_exposure = metadata_pair(metadata, "actual_exposure")
            actual_gain = metadata_pair(metadata, "gain_level", 1)
            actual_auto_exposure = metadata_switch_pair(metadata, "auto_exposure")
            actual_emitter = metadata_switch_pair(metadata, "frame_emitter_mode")
            metadata_signature = (
                metadata_bucket_pair(metadata, "actual_exposure", 1000.0),
                metadata_bucket_pair(metadata, "gain_level", 2.0),
                actual_auto_exposure,
                actual_emitter,
            )
            if metadata_signature != STATE.last_logged_metadata_signature:
                STATE.last_logged_metadata_signature = metadata_signature
                if metadata_signature == ((None, None), (None, None), "—", "—"):
                    STATE.add_log("=== D435i 實際狀態（Metadata） ===")
                    STATE.add_log("Actual status: metadata 尚未收到可用數值")
                else:
                    STATE.add_log("=== D435i 實際狀態（Metadata） ===")
                    STATE.add_log(f"Actual Exposure L/R: {actual_exposure} us")
                    STATE.add_log(f"Actual Gain L/R: {actual_gain}")
                    STATE.add_log(f"Auto Exposure L/R: {actual_auto_exposure}")
                    STATE.add_log(f"Emitter L/R: {actual_emitter}")
            if flags != STATE.last_logged_quality_signature:
                STATE.last_logged_quality_signature = flags
                STATE.add_log("=== Quality / Geometry ===")
                STATE.add_log(f"Image Flags: {', '.join(flags) if flags else 'none'}")
                STATE.add_log(f"Motion Label: {STATE.r2_motion_label}")
                gate = STATE.turn_translation_candidate_status
                STATE.add_log(f"Geometry Gate: {gate if gate in {'PASS', 'REVIEW', 'FAIL'} else 'PENDING'}")
            if STATE.motion_recording:
                STATE.motion_quality_samples.append(
                    {
                        "elapsed_s": time.monotonic() - STATE.motion_started_at,
                        "flags": list(data.get("flags", [])),
                        "left": data.get("left", {}).get("window", {}),
                        "right": data.get("right", {}).get("window", {}),
                        "metadata": data.get("metadata", {}),
                    }
                )
                STATE.motion_quality_samples = STATE.motion_quality_samples[-2400:]
        append_run_trace("image_quality", data)
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        STATE.add_log(f"image quality parse failed: {exc}")


def quality_worker(process: subprocess.Popen[str]) -> None:
    assert process.stdout is not None
    for raw_line in process.stdout:
        handle_quality_line(raw_line.strip())
    process.wait()
    with STATE.lock:
        if STATE.quality_process is process:
            STATE.quality_process = None
        if STATE.r2_running:
            STATE.image_quality = {"connected": False, "flags": []}
            STATE.add_log("影像品質診斷已中斷")


def start_telemetry(mode: str = "r2") -> None:
    with STATE.lock:
        if STATE.telemetry_supervisor_running:
            return
        STATE.telemetry_mode = mode
        STATE.telemetry_supervisor_running = True
    threading.Thread(target=telemetry_supervisor, daemon=True).start()


def start_quality() -> None:
    with STATE.lock:
        if STATE.quality_process is not None and STATE.quality_process.poll() is None:
            return
    process = subprocess.Popen(
        active_runtime_command("quality"),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    with STATE.lock:
        STATE.quality_process = process
    threading.Thread(target=quality_worker, args=(process,), daemon=True).start()


def r2_start_worker(
    mode: str, mapping: str, profile: str, lighting: str,
    scene: str, motion: str, diagnostics: str, estimator_stack: str,
    emitter_override: str,
) -> None:
    try:
        log_requested_runtime(
            mode, mapping, profile, lighting, diagnostics, scene, motion,
            estimator_stack, emitter_override,
        )
        runtime_command = (
            remote_rplidar_command(
                "start", mode, profile, lighting, diagnostics, estimator_stack,
                LIDAR_YAW, emitter_override, LIDAR_X, LIDAR_Y, LIDAR_Z,
            )
            if mapping == "mapping"
            else remote_r2_command(
                "start", mode, mapping, "no_reset", "slam", profile,
                lighting, diagnostics, emitter_override,
            )
        )
        result = subprocess.run(
            runtime_command,
            capture_output=True,
            text=True,
            timeout=120,
        )
        output = (result.stdout + result.stderr).strip()
        if output:
            for line in output.splitlines()[-30:]:
                STATE.add_log(line)
        with STATE.lock:
            STATE.r2_running = result.returncode == 0
            STATE.r2_busy = False
            STATE.r2_mode = mode
            STATE.r2_mapping = mapping
            STATE.r2_profile = profile
            STATE.r2_lighting = lighting
            STATE.r2_emitter_override = emitter_override
            STATE.r2_diagnostics = diagnostics
            STATE.r2_estimator = estimator_stack
            STATE.r2_scene_label = scene
            STATE.r2_motion_label = motion
            if result.returncode == 0:
                STATE.r2_status = (
                    "2D SLAM（cuVSLAM + A2M12 + slam_toolbox）已啟動，正在連線…"
                    if mapping == "mapping" else "R2 容器已啟動，正在連接 odom…"
                )
            else:
                STATE.r2_status = "2D SLAM 啟動失敗" if mapping == "mapping" else "R2 啟動失敗"
        if result.returncode == 0:
            append_run_trace("r2_remote_started", {
                "runtime": "real_d435i_rplidar" if mapping == "mapping" else "real_d435i_r2",
                "scene": scene, "motion": motion, "diagnostics": diagnostics,
                "estimator_stack": estimator_stack,
                "emitter_override": emitter_override,
                "lidar_x_m": LIDAR_X,
                "lidar_y_m": LIDAR_Y,
                "lidar_z_m": LIDAR_Z,
                "lidar_yaw_rad": LIDAR_YAW,
            })
            start_telemetry()
            start_quality()
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(str(exc))
        with STATE.lock:
            STATE.r2_running = False
            STATE.r2_busy = False
            STATE.r2_status = "2D SLAM 啟動失敗" if mapping == "mapping" else "R2 啟動失敗"


def start_r2(
    mode: str, mapping: str, profile: str, lighting: str,
    scene: str = "scene_unknown", motion: str = "other",
    diagnostics: str = "standard", estimator_stack: str = "cuvslam",
    emitter_override: str = "profile",
) -> str:
    if mode not in {"stereo", "imu"}:
        return "R2 模式無效"
    if mapping not in {"odom", "mapping"}:
        return "R2 工作模式無效"
    if profile not in {"640x360x30", "848x480x30"}:
        return "R2 影像 profile 無效"
    if lighting not in {
        "official", "baseline", "robust", "low_light", "low_light_motion",
        "high_contrast",
    }:
        return "R2 光照策略無效"
    if emitter_override not in {"profile", "on", "off"}:
        return "IR projector A/B 設定無效"
    if diagnostics not in {"standard", "features"}:
        return "R2 特徵診斷模式無效"
    if estimator_stack not in {"cuvslam", "ekf_shadow"}:
        return "R2 估計器模式無效"
    if estimator_stack == "ekf_shadow" and mapping != "mapping":
        return "LiDAR + EKF shadow 只能在 mapping 工作模式使用"
    scene = " ".join(scene.split())[:48] or "scene_unknown"
    if motion not in {"forward_left_turn", "forward_right_turn", "pure_turn", "straight", "closed_loop", "other"}:
        motion = "other"
    with STATE.lock:
        if STATE.localization_running or STATE.localization_busy:
            return "定位測試正在執行，請先停止定位"
        if STATE.r2_running or STATE.r2_busy:
            return "R2 容器已在執行"
        STATE.r2_busy = True
        STATE.r2_mode = mode
        STATE.r2_mapping = mapping
        STATE.r2_profile = profile
        STATE.r2_lighting = lighting
        STATE.r2_emitter_override = emitter_override
        STATE.r2_diagnostics = diagnostics
        STATE.r2_estimator = estimator_stack
        STATE.r2_scene_label = scene
        STATE.r2_motion_label = motion
        STATE.r2_started_at = time.strftime("%Y-%m-%d %H:%M:%S")
        STATE.last_mapping_summary = ""
        STATE.run_trace_file = str(QUALITY_LOG_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_r2_trace.jsonl")
        STATE.run_trace_records = 0
        STATE.run_session_file = ""
        STATE.r2_recording = False
        STATE.r2_recording_name = ""
        STATE.r2_recording_mode = "light"
        STATE.r2_recording_status = "尚未錄製建圖資料"
        STATE.r2_download_status = "輕量模式下載：CameraInfo、IMU、TF、odom、VO pose、SLAM path、landmarks、pose graph、loop closure 與 GUI 摘要；特徵診斷開啟時才有 observations；完整模式才包含原始 IR 影像。"
        STATE.logs = []
        STATE.add_log(
            f"R2 process started mode={mode} mapping={mapping} "
            f"profile={profile} lighting={lighting} diagnostics={diagnostics} "
            f"estimator={estimator_stack} emitter_override={emitter_override} "
            f"lidar_xyz=({LIDAR_X},{LIDAR_Y},{LIDAR_Z}) lidar_yaw={LIDAR_YAW} "
            f"scene={scene} motion={motion}"
        )
        STATE.r2_status = "正在啟動 2D SLAM（cuVSLAM + A2M12 + slam_toolbox）…" if mapping == "mapping" else "正在啟動 R2 cuVSLAM…"
    reset_r2_live_state()
    append_run_trace("r2_start_requested", {
        "mode": mode, "mapping": mapping, "profile": profile, "lighting": lighting,
        "emitter_override": emitter_override,
        "scene": scene, "motion": motion, "diagnostics": diagnostics,
        "estimator_stack": estimator_stack,
        "requested_runtime": requested_runtime_settings(
            mode, mapping, profile, lighting, diagnostics, scene, motion,
            estimator_stack, emitter_override,
        ),
    })
    threading.Thread(
        target=r2_start_worker,
        args=(mode, mapping, profile, lighting, scene, motion, diagnostics, estimator_stack, emitter_override),
        daemon=True,
    ).start()
    return "正在啟動 2D SLAM（cuVSLAM + A2M12 + slam_toolbox）…" if mapping == "mapping" else "正在啟動 R2 cuVSLAM…"


def reset_localization_live_state() -> None:
    with STATE.lock:
        STATE.localization_pose = {
            "available": False, "status": "WAIT_AMCL", "frame_id": "",
            "x_m": None, "y_m": None, "yaw_deg": None,
            "covariance_x_m2": None, "covariance_y_m2": None,
            "covariance_yaw_rad2": None,
        }
        STATE.mapping_map = {}
        STATE.mapping_status = "尚未收到 /map"
        STATE.mapping_path = []
        STATE.mapping_robot_pose = []
        STATE.lidar_scan = {
            "status": "尚未收到 /scan", "frame_id": "", "points": [],
            "source_points": 0, "display_points": 0,
            "alignment": {"status": "WAIT"}, "timing": {"status": "WAIT"},
        }


def localization_start_worker() -> None:
    try:
        result = subprocess.run(
            remote_localization_command("start"),
            capture_output=True,
            text=True,
            timeout=120,
        )
        output = (result.stdout + result.stderr).strip()
        if output:
            for line in output.splitlines()[-30:]:
                STATE.add_log(line)
        with STATE.lock:
            STATE.localization_busy = False
            STATE.localization_running = result.returncode == 0
            STATE.localization_started_at = time.strftime("%Y-%m-%d %H:%M:%S") if result.returncode == 0 else ""
            STATE.localization_status = (
                "定位節點已啟動，等待 /map、/amcl_pose、/scan 與 TF…"
                if result.returncode == 0 else "定位啟動失敗"
            )
        if result.returncode == 0:
            append_run_trace("localization_remote_started", {
                "map": "20260924_092630_current_cuvslam_a2m12",
                "tf_owner": "candidate_amcl",
                "runtime": "experimental_candidate_localization",
            })
            start_telemetry("localization")
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(str(exc))
        with STATE.lock:
            STATE.localization_running = False
            STATE.localization_busy = False
            STATE.localization_status = "定位啟動失敗"


def start_localization() -> str:
    with STATE.lock:
        if STATE.r2_running or STATE.r2_busy:
            return "請先停止 R2/SLAM；避免 map→odom owner 衝突"
        if STATE.localization_running or STATE.localization_busy:
            return "定位節點已在執行或啟動中"
        STATE.localization_busy = True
        STATE.localization_status = "正在啟動實驗定位節點…"
        STATE.telemetry_reconnects = 0
        STATE.telemetry_last_data_monotonic = 0.0
    reset_localization_live_state()
    threading.Thread(target=localization_start_worker, daemon=True).start()
    return "正在啟動實驗定位節點…"


def localization_stop_worker() -> None:
    try:
        result = subprocess.run(
            remote_localization_command("stop"),
            capture_output=True,
            text=True,
            timeout=60,
        )
        output = (result.stdout + result.stderr).strip()
        if output:
            for line in output.splitlines()[-20:]:
                STATE.add_log(line)
        with STATE.lock:
            STATE.localization_busy = False
            STATE.localization_status = "定位已停止" if result.returncode == 0 else "定位停止完成或容器不存在"
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(str(exc))
        with STATE.lock:
            STATE.localization_busy = False
            STATE.localization_status = "定位停止失敗"


def stop_localization() -> str:
    process = STATE.telemetry_process
    if process is not None and process.poll() is None:
        process.terminate()
    with STATE.lock:
        if not STATE.localization_running and not STATE.localization_busy:
            return "定位目前沒有執行"
        STATE.localization_running = False
        STATE.localization_busy = True
        STATE.localization_status = "正在停止定位節點…"
        STATE.telemetry_process = None
    reset_localization_live_state()
    threading.Thread(target=localization_stop_worker, daemon=True).start()
    return "正在停止定位節點…"


def recording_run_name(record_mode: str) -> str:
    with STATE.lock:
        mode = STATE.r2_mode
        mapping = STATE.r2_mapping
        scene = "_".join(STATE.r2_scene_label.split())[:32] or "scene_unknown"
        motion = STATE.r2_motion_label
    return f"gui_{time.strftime('%Y%m%d_%H%M%S')}_{mode}_{mapping}_{scene}_{motion}_{record_mode}"


def stop_remote_r2_recording(run_name: str) -> bool:
    try:
        result = subprocess.run(
            active_runtime_command("record_stop", run_name),
            capture_output=True,
            text=True,
            timeout=60,
        )
        output = (result.stdout + result.stderr).strip()
        if output:
            for line in output.splitlines()[-10:]:
                STATE.add_log(line)
        return result.returncode == 0
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(f"建圖錄製停止失敗：{exc}")
        return False


def r2_recording_failure_message(output: str) -> str:
    """Turn remote recorder failures into a concise GUI status."""
    if "reason=low_disk_space" in output:
        details: dict[str, str] = {}
        for token in output.replace("\n", " ").split():
            if "=" in token:
                key, value = token.split("=", 1)
                details[key] = value
        available = details.get("available_mb", "?")
        required = details.get("required_mb", "?")
        mode = details.get("mode", "unknown")
        return f"錄製失敗：Jetson 空間不足（{mode} 可用 {available} MB，至少需要 {required} MB）"
    if "No space left on device" in output:
        return "錄製失敗：Jetson 磁碟已滿，請清理舊錄製後再試"
    if "R2 container is not running" in output:
        return "錄製失敗：R2 尚未啟動"
    if "RPLIDAR mapping container is not running" in output:
        return "錄製失敗：2D SLAM 尚未啟動"
    if "recording_start_failed" in output:
        return "錄製失敗：rosbag 未成功啟動，請查看工作記錄"
    return "建圖錄製啟動失敗；請查看工作記錄"


def r2_record_start_worker(run_name: str, record_mode: str) -> None:
    try:
        result = subprocess.run(
            active_runtime_command("record_start", run_name, record_mode),
            capture_output=True,
            text=True,
            timeout=60,
        )
        output = (result.stdout + result.stderr).strip()
        if output:
            for line in output.splitlines()[-10:]:
                STATE.add_log(line)
        with STATE.lock:
            if result.returncode == 0:
                STATE.r2_recording = True
                STATE.r2_recording_name = run_name
                STATE.r2_recording_mode = record_mode
                STATE.r2_recording_status = f"錄製中（{record_mode}）：{run_name}"
            else:
                STATE.r2_recording_status = r2_recording_failure_message(output)
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(f"建圖錄製啟動失敗：{exc}")
        with STATE.lock:
            STATE.r2_recording_status = f"建圖錄製啟動失敗：{exc}"


def start_r2_recording(record_mode: str) -> str:
    if record_mode not in {"light", "full"}:
        return "錄製模式無效"
    with STATE.lock:
        if not STATE.r2_running:
            return "請先啟動目前工作"
        if STATE.r2_recording:
            return f"建圖資料錄製中：{STATE.r2_recording_name}"
        run_name = recording_run_name(record_mode)
        STATE.r2_recording_mode = record_mode
        STATE.r2_recording_status = f"正在啟動{record_mode}建圖資料錄製…"
    threading.Thread(target=r2_record_start_worker, args=(run_name, record_mode), daemon=True).start()
    return "正在啟動建圖資料錄製…請在移動前等待狀態變成錄製中"


def r2_record_stop_worker(run_name: str) -> None:
    ok = stop_remote_r2_recording(run_name)
    with STATE.lock:
        STATE.r2_recording = False
        STATE.r2_recording_status = f"已停止錄製：{run_name}" if ok else "建圖錄製停止需確認"


def stop_r2_recording() -> str:
    with STATE.lock:
        if not STATE.r2_recording:
            return "目前沒有進行中的建圖錄製"
        run_name = STATE.r2_recording_name
        STATE.r2_recording_status = "正在停止建圖資料錄製…"
    threading.Thread(target=r2_record_stop_worker, args=(run_name,), daemon=True).start()
    return "正在停止建圖資料錄製…"


def r2_download_worker(run_name: str, summary: str) -> None:
    LOCAL_R2_DATA_DIR.mkdir(parents=True, exist_ok=True)
    local_run = LOCAL_R2_DATA_DIR / run_name
    remote = f"{USER}@{HOST}:{active_runtime_data_dir()}/{run_name}"
    command = [
        "scp", "-r", "-P", "22", "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=8", remote, str(LOCAL_R2_DATA_DIR),
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=900)
        if result.returncode != 0:
            message = result.stderr.strip() or "建圖資料下載失敗"
            with STATE.lock:
                STATE.r2_download_status = message
            STATE.add_log(message)
            return
        local_run.mkdir(parents=True, exist_ok=True)
        (local_run / "gui_mapping_summary.txt").write_text(summary + "\n", encoding="utf-8")
        with STATE.lock:
            STATE.r2_download_status = f"已下載至：{local_run}"
        STATE.add_log(f"建圖資料已下載：{local_run}")
    except Exception as exc:  # noqa: BLE001
        with STATE.lock:
            STATE.r2_download_status = f"建圖資料下載失敗：{exc}"
        STATE.add_log(f"建圖資料下載失敗：{exc}")


def download_r2_recording() -> str:
    with STATE.lock:
        if STATE.r2_recording:
            return "請先停止建圖錄製，再下載資料"
        run_name = STATE.r2_recording_name
        summary = STATE.last_mapping_summary or STATE.mapping_summary_text()
        if not run_name:
            return "尚未有可下載的建圖錄製；下一次請先按開始錄製建圖資料"
        STATE.r2_download_status = f"正在下載 {run_name}，rosbag 可能需要幾分鐘…"
    threading.Thread(target=r2_download_worker, args=(run_name, summary), daemon=True).start()
    return f"正在下載 {run_name}…"


def r2_stop_worker() -> None:
    cleanup_ok = False
    with STATE.lock:
        recording = STATE.r2_recording
        recording_name = STATE.r2_recording_name
    if recording and recording_name:
        stop_remote_r2_recording(recording_name)
        with STATE.lock:
            STATE.r2_recording = False
            STATE.r2_recording_status = f"已停止錄製：{recording_name}"
    try:
        result = subprocess.run(
            active_runtime_command("cleanup"),
            capture_output=True,
            text=True,
            timeout=60,
        )
        cleanup_ok = result.returncode == 0
        output = (result.stdout + result.stderr).strip()
        if output:
            for line in output.splitlines()[-20:]:
                STATE.add_log(line)
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(str(exc))
    rviz_reset_ok = False
    try:
        rviz_result = subprocess.run(
            remote_rviz_command("reset"),
            capture_output=True,
            text=True,
            timeout=60,
        )
        rviz_reset_ok = rviz_result.returncode == 0
        rviz_output = (rviz_result.stdout + rviz_result.stderr).strip()
        if rviz_output:
            for line in rviz_output.splitlines()[-20:]:
                STATE.add_log(line)
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(f"RViz reset failed: {exc}")
    reset_r2_live_state()
    with STATE.lock:
        STATE.r2_running = False
        STATE.r2_busy = False
        if cleanup_ok and rviz_reset_ok:
            STATE.r2_status = "目前 runtime 已停止；R2 與 RViz 畫面都已清除"
        elif cleanup_ok:
            STATE.r2_status = "R2 已停止；R2 已清除，但 RViz 畫面重置需確認"
        else:
            STATE.r2_status = "R2 停止完成，但遠端清理需確認"
        STATE.add_log("R2 cleanup completed; saved JSON records were preserved")


def stop_r2(wait: bool = False) -> str:
    with STATE.lock:
        if STATE.r2_busy:
            return "R2 正在啟動或停止，請稍候"
        if STATE.odom_samples:
            STATE.last_mapping_summary = STATE.mapping_summary_text()
        final_assessment = STATE.quality_assessment()
        final_flags = final_assessment.get("input_flags", [])
        final_flags_text = ", ".join(str(item) for item in final_flags) if final_flags else "none"
        final_geometry = str(final_assessment.get("geometry_gate", "PENDING"))
        final_geometry = final_geometry if final_geometry in {"PASS", "REVIEW", "FAIL"} else "PENDING"
        STATE.add_log("=== Quality / Geometry Final ===")
        STATE.add_log(f"Image Flags: {final_flags_text}")
        STATE.add_log(f"Motion Label: {STATE.r2_motion_label}")
        STATE.add_log(f"Geometry Gate: {final_geometry}")
        worst_geometry = final_assessment.get("worst_geometry", {})
        if isinstance(worst_geometry, dict):
            STATE.add_log(
                "Worst Geometry: "
                f"gate={worst_geometry.get('gate', '未偵測')} "
                f"yaw_delta={float(worst_geometry.get('yaw_delta_deg', 0.0)):.1f} deg "
                f"max_xy={float(worst_geometry.get('max_xy_displacement_m', 0.0)):.3f} m"
            )
        STATE.add_log(
            f"Assessment: {final_assessment.get('overall', 'WAIT')} "
            f"score={final_assessment.get('score', '—')}"
        )
    append_run_trace("r2_stop_requested", {"odom_samples": STATE.odom_samples})
    try:
        session_file = save_r2_session()
        if session_file:
            STATE.add_log(f"R2 session 已保存：{session_file}")
    except (OSError, TypeError, ValueError) as exc:
        STATE.add_log(f"R2 session 保存失敗：{exc}")
    process = STATE.telemetry_process
    if process is not None and process.poll() is None:
        process.terminate()
    quality_process = STATE.quality_process
    if quality_process is not None and quality_process.poll() is None:
        quality_process.terminate()
    with STATE.lock:
        STATE.r2_running = False
        STATE.r2_busy = True
        STATE.r2_status = "正在停止 R2 並清除即時工作區…"
        STATE.telemetry_process = None
        STATE.quality_process = None
    reset_r2_live_state()
    worker = threading.Thread(target=r2_stop_worker, daemon=True)
    worker.start()
    if wait:
        worker.join(timeout=150)
    return "正在停止 R2 並清除即時工作區…"


def start_motion_record() -> str:
    with STATE.lock:
        if not STATE.r2_running:
            return "請先啟動目前工作"
        if STATE.motion_recording:
            return "位移記錄已在進行"
        STATE.motion_recording = True
        STATE.motion_status = "記錄中…請移動"
        STATE.motion_started_at = time.monotonic()
        STATE.motion_start_position = list(STATE.odom_position) if STATE.odom_samples else None
        STATE.motion_start_path_length_m = STATE.odom_path_length_m if STATE.odom_samples else None
        STATE.motion_start_samples = STATE.odom_samples
        STATE.motion_duration_s = 0.0
        STATE.motion_displacement_m = 0.0
        STATE.motion_path_length_m = 0.0
        STATE.motion_max_displacement_m = 0.0
        STATE.motion_samples = 0
        STATE.motion_quality_samples = []
        STATE.motion_record_file = ""
    return "開始記錄；請移動"


def stop_motion_record() -> str:
    payload: dict[str, object] | None = None
    with STATE.lock:
        if not STATE.motion_recording:
            return "目前沒有進行中的位移記錄"
        STATE.motion_duration_s = time.monotonic() - STATE.motion_started_at
        STATE.motion_recording = False
        QUALITY_LOG_DIR.mkdir(parents=True, exist_ok=True)
        output_path = QUALITY_LOG_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_r2_motion_quality.json"
        payload = {
            "hardware": "Intel RealSense D435i",
            "mode": STATE.r2_mode,
            "mapping": STATE.r2_mapping,
            "profile": STATE.r2_profile,
            "lighting": STATE.r2_lighting,
            "duration_s": STATE.motion_duration_s,
            "displacement_m": STATE.motion_displacement_m,
            "path_length_m": STATE.motion_path_length_m,
            "max_displacement_m": STATE.motion_max_displacement_m,
            "odom_samples": STATE.motion_samples,
            "odom_path": list(STATE.odom_path),
            "odom_path_length_total_m": STATE.odom_path_length_m,
            "odom_samples_total": STATE.odom_samples,
            "vo_valid_ratio": STATE.odom_vo_valid_ratio,
            "vo_ratio_source": STATE.odom_vo_ratio_source,
            "vo_ratio_samples": STATE.odom_vo_ratio_samples,
            "vo_valid_samples": STATE.odom_vo_valid_samples,
            "odom_rate_hz": STATE.odom_rate_hz,
            "odom_gap_p95_ms": STATE.odom_gap_p95_ms,
            "odom_rate_basis": STATE.odom_rate_basis,
            "odom_timestamp_span_s": STATE.odom_timestamp_span_s,
            "turn_jitter": STATE.turn_jitter_summary(),
            "quality_assessment": STATE.quality_assessment(),
            "quality_samples": list(STATE.motion_quality_samples),
        }
        STATE.motion_record_file = str(output_path)
        STATE.motion_status = f"已停止並保存：{output_path.name}"
    assert payload is not None
    try:
        output_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=True) + "\n")
    except OSError as exc:
        STATE.add_log(f"影像品質記錄保存失敗：{exc}")
        return "已停止，但影像品質記錄保存失敗"
    return f"已停止並保存：{output_path.name}"


def clear_motion_record() -> str:
    with STATE.lock:
        if STATE.motion_recording:
            return "請先停止目前的位移記錄"
        STATE.motion_status = "尚未記錄"
        STATE.motion_started_at = 0.0
        STATE.motion_start_position = None
        STATE.motion_start_path_length_m = None
        STATE.motion_start_samples = 0
        STATE.motion_duration_s = 0.0
        STATE.motion_displacement_m = 0.0
        STATE.motion_path_length_m = 0.0
        STATE.motion_max_displacement_m = 0.0
        STATE.motion_samples = 0
        STATE.motion_quality_samples = []
        STATE.motion_record_file = ""
    return "已清除位移記錄"


def clear_process_log() -> str:
    """Clear only the in-memory process timeline; preserve evidence files."""
    with STATE.lock:
        STATE.logs.clear()
    return "已清除工作記錄；JSONL、session 與摘要檔案仍保留"


def run_simple(command: list[str], success: str, failure: str, mark_rig_running: bool = False) -> None:
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=100)
        output = (result.stdout + result.stderr).strip()
        if output:
            for line in output.splitlines()[-30:]:
                STATE.add_log(line)
        with STATE.lock:
            STATE.status = success if result.returncode == 0 else failure
            if mark_rig_running:
                STATE.rig_running = result.returncode == 0
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(str(exc))
        with STATE.lock:
            STATE.status = failure


def start_rig() -> str:
    with STATE.lock:
        if STATE.rig_running:
            return "實機容器已在執行"
        STATE.status = "正在啟動 Jetson D435i + cuVSLAM…"
    threading.Thread(target=run_simple, args=(remote_r1p5_command("start"), "實機容器已啟動，可開始 phase", "啟動失敗", True), daemon=True).start()
    return "正在啟動 Jetson D435i + cuVSLAM…"


def stop_rig() -> str:
    process = STATE.phase_process
    if process is not None and process.poll() is None:
        process.terminate()
    with STATE.lock:
        STATE.phase = ""; STATE.phase_process = None; STATE.status = "正在停止實機容器…"; STATE.rig_running = False
    threading.Thread(target=run_simple, args=(remote_r1p5_command("stop"), "實機容器已停止", "停止完成或容器不存在"), daemon=True).start()
    return "正在停止實機容器…"


def handle_phase_line(line: str) -> None:
    if line.startswith("LIVE "):
        try:
            data = json.loads(line[5:])
            with STATE.lock:
                STATE.gyro = data["gyro_rad_s"]; STATE.peaks = data["peak_abs_gyro_rad_s"]
                STATE.dominant = data["dominant_axis"]; STATE.elapsed = data["elapsed_s"]
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            STATE.add_log(line)
    elif line.startswith("{"):
        try:
            result = json.loads(line)
            summary = (f"{result.get('label')}  samples={result.get('samples')}  dominant={result.get('dominant_axis')}\n"
                       f"integrated={result.get('integrated_gyro_rad')}\npeak={result.get('peak_abs_gyro_rad_s')}")
            with STATE.lock:
                STATE.results.append(summary); STATE.results = STATE.results[-20:]
        except json.JSONDecodeError:
            STATE.add_log(line)
    elif line:
        STATE.add_log(line)


def download_result(output_name: str) -> None:
    LOCAL_LOG_DIR.mkdir(parents=True, exist_ok=True)
    remote = f"{USER}@{HOST}:{REMOTE_DATA_DIR}/{output_name}"
    command = ["scp", "-P", "22", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", remote, str(LOCAL_LOG_DIR / output_name)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    STATE.add_log(f"已下載 {LOCAL_LOG_DIR / output_name}" if result.returncode == 0 else (result.stderr.strip() or "下載結果失敗"))


def phase_worker(process: subprocess.Popen[str], output_name: str) -> None:
    assert process.stdout is not None
    for raw_line in process.stdout:
        handle_phase_line(raw_line.strip())
    return_code = process.wait()
    try:
        download_result(output_name)
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(f"下載結果失敗：{exc}")
    with STATE.lock:
        STATE.phase = ""; STATE.phase_process = None
        STATE.status = "phase 完成；可選擇下一個方向" if return_code == 0 else "phase 失敗，請查看工作記錄"


def start_phase(phase: str, duration: float) -> str:
    with STATE.lock:
        if not STATE.rig_running: return "請先啟動實機測試"
        if STATE.phase: return "目前 phase 尚未完成"
        STATE.phase = phase; STATE.output_name = f"{time.strftime('%Y%m%d_%H%M%S')}_{phase}.json"
        STATE.status = f"收集 {PHASES[phase][0]}，{duration:.1f} 秒"; STATE.gyro = [0.0, 0.0, 0.0]
        STATE.peaks = [0.0, 0.0, 0.0]; STATE.dominant = "—"; STATE.elapsed = 0.0; output_name = STATE.output_name
    process = subprocess.Popen(remote_r1p5_command("phase", phase, f"{duration:g}", output_name), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    with STATE.lock: STATE.phase_process = process
    threading.Thread(target=phase_worker, args=(process, output_name), daemon=True).start()
    return f"收集 {PHASES[phase][0]}，{duration:.1f} 秒"


class Handler(BaseHTTPRequestHandler):
    def log_message(self, _format: str, *_args: object) -> None: return

    def send_json(self, payload: dict[str, object], code: int = 200) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode(); self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8"); self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data)

    def body(self) -> dict[str, object]:
        length = int(self.headers.get("Content-Length", "0")); return json.loads(self.rfile.read(length) or b"{}")

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/api/state": self.send_json(STATE.snapshot()); return
        if path == "/":
            data = HTML.encode(); self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data); return
        self.send_json({"error": "not found"}, 404)

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            data = self.body()
            if path == "/api/start": self.send_json({"message": start_rig()})
            elif path == "/api/stop": self.send_json({"message": stop_rig()})
            elif path == "/api/r2/start": self.send_json({"message": start_r2(str(data.get("mode", "stereo")), str(data.get("mapping", "odom")), str(data.get("profile", "640x360x30")), str(data.get("lighting", "robust")), str(data.get("scene", "scene_unknown")), str(data.get("motion", "other")), str(data.get("diagnostics", "standard")), str(data.get("estimator", "cuvslam")), str(data.get("emitter", "profile")))})
            elif path == "/api/r2/stop": self.send_json({"message": stop_r2()})
            elif path == "/api/localization/start": self.send_json({"message": start_localization()})
            elif path == "/api/localization/stop": self.send_json({"message": stop_localization()})
            elif path == "/api/r2/record/start": self.send_json({"message": start_r2_recording(str(data.get("mode", "light")))})
            elif path == "/api/r2/record/stop": self.send_json({"message": stop_r2_recording()})
            elif path == "/api/r2/record/download": self.send_json({"message": download_r2_recording()})
            elif path == "/api/rviz/open": self.send_json({"message": open_rviz()})
            elif path == "/api/r2/motion/start": self.send_json({"message": start_motion_record()})
            elif path == "/api/r2/motion/stop": self.send_json({"message": stop_motion_record()})
            elif path == "/api/r2/motion/clear": self.send_json({"message": clear_motion_record()})
            elif path == "/api/logs/clear": self.send_json({"message": clear_process_log()})
            elif path == "/api/phase":
                phase = str(data.get("phase", "")); duration = float(data.get("duration", 10))
                if phase not in PHASES or not 3 <= duration <= 60: self.send_json({"message": "phase 或秒數無效"}, 400)
                else: self.send_json({"message": start_phase(phase, duration)})
            else: self.send_json({"error": "not found"}, 404)
        except Exception as exc:  # noqa: BLE001
            self.send_json({"message": str(exc)}, 500)


def bind_gui_server(requested_port: int) -> tuple[ThreadingHTTPServer, int]:
    """Bind the GUI, moving to the next loopback port if the default is busy."""
    candidates = [requested_port] if requested_port == 0 else range(requested_port, requested_port + 20)
    last_error: OSError | None = None
    for port in candidates:
        try:
            return ThreadingHTTPServer(("127.0.0.1", port), Handler), port
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE:
                raise
            last_error = exc
    raise OSError(
        f"No available local GUI port in {requested_port}-{requested_port + 19}"
    ) from last_error


def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("--port", type=int, default=8765); parser.add_argument("--open-browser", action="store_true"); args = parser.parse_args()
    server, bound_port = bind_gui_server(args.port); url = f"http://127.0.0.1:{bound_port}/"
    if bound_port != args.port:
        print(f"Port {args.port} is busy; using {bound_port} instead.", flush=True)
    print(f"Real D435i GUI: {url}", flush=True)
    if args.open_browser: threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        stop_rig()
        stop_localization()
        stop_r2(wait=True)
        server.server_close()


if __name__ == "__main__": main()
