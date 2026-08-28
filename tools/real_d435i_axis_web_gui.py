#!/usr/bin/env python3
"""Local browser GUI for the real D435i R1.5 axis test."""

from __future__ import annotations

import argparse
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


PROJECT_ROOT = Path(__file__).resolve().parents[1]


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


LOCAL_ENV = load_local_env(PROJECT_ROOT / "config" / "local.env")


def local_setting(name: str, default: str) -> str:
    return os.environ.get(name, LOCAL_ENV.get(name, default))


REMOTE_SCRIPT = "/home/tseng/isaac_ros_ws/tools/run_real_d435i_r1p5_axis.sh"
REMOTE_R2_SCRIPT = "/home/tseng/isaac_ros_ws/tools/run_real_d435i_r2.sh"
REMOTE_RVIZ_VIEW_SCRIPT = "/home/tseng/isaac_ros_ws/tools/open_real_d435i_rviz_view.sh"
REMOTE_DATA_DIR = "/home/tseng/isaac_ros_data/experiments/real_d435i_r1p5_gui"
REMOTE_R2_DATA_DIR = "/home/tseng/isaac_ros_data/experiments/real_d435i_r2"
LOCAL_LOG_DIR = PROJECT_ROOT / "logs" / "real_d435i_r1p5_gui"
QUALITY_LOG_DIR = PROJECT_ROOT / "logs" / "real_d435i_quality"
LOCAL_R2_DATA_DIR = PROJECT_ROOT / "logs" / "real_d435i_r2_downloads"
CORRECTION_LOG_PATH = QUALITY_LOG_DIR / "20260825_r2_correction_log.md"
HOST = local_setting("CUVSLAM_JETSON_HOST", "192.168.55.1")
USER = local_setting("CUVSLAM_JETSON_USER", "tseng")
VNC_PASSWORD = local_setting("CUVSLAM_VNC_PASSWORD", "")
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
  <section class="panel r2-panel">
    <div class="section-heading">
      <h2>R2 即時 odom（RViz 觀察區）</h2>
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
          <option value="low_light">Low-light alias：去噪 + 開 IR projector</option>
          <option value="high_contrast">診斷：手動曝光 8000（不建議正式建圖）</option>
          <option value="baseline">Baseline：原始設定</option>
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
        <button class="primary" id="r2-start" onclick="startR2()">啟動 R2 cuVSLAM</button>
        <button class="danger" id="r2-stop" onclick="stopR2()">停止 R2 並清除</button>
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
        <div class="r2-note">本次記錄會保存整段 XY 路徑、整段 yaw、轉向片段、影像品質快照與事件時間戳。按「停止 R2 並清除」只清除即時工作區，已保存的 JSONL／session 檔案不會刪除。</div>
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
        <div class="odom-metric"><span>vo_state</span><strong id="odom-state">—</strong></div>
        <div class="r2-note">資料來源：/visual_slam/tracking/odometry、/visual_slam/status。軌跡圖為即時 XY 投影。</div>
      </div>
      <canvas id="odom-canvas" class="odom-canvas" width="820" height="300"></canvas>
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
  const scene = document.getElementById('r2-scene-label').value.trim() || 'scene_unknown';
  const motion = document.getElementById('r2-motion-label').value;
  const diagnostics = document.getElementById('r2-diagnostics').value;
  post('/api/r2/start', {mode, mapping, profile, lighting, scene, motion, diagnostics}).then(x => document.getElementById('r2-status').textContent = x.message || 'R2 啟動中');
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
function drawOdomPath(points, startYawDeg, currentYawDeg) {
  const canvas = document.getElementById('odom-canvas');
  const ctx = canvas.getContext('2d');
  const width = canvas.width, height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#0f172a'; ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = '#334155'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(width / 2, 0); ctx.lineTo(width / 2, height); ctx.moveTo(0, height / 2); ctx.lineTo(width, height / 2); ctx.stroke();
  if (!points || points.length < 1) return;
  const xs = points.map(p => p[0]), ys = points.map(p => p[1]);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
  const span = Math.max(maxX - minX, maxY - minY, 0.4);
  const pad = 35, scale = Math.min((width - 2 * pad) / span, (height - 2 * pad) / span);
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
  const map = p => [width / 2 + (p[0] - cx) * scale, height / 2 - (p[1] - cy) * scale];
  ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 3; ctx.beginPath();
  points.forEach((p, i) => { const q = map(p); if (i === 0) ctx.moveTo(q[0], q[1]); else ctx.lineTo(q[0], q[1]); });
  ctx.stroke();
  const start = map(points[0]), end = map(points[points.length - 1]);
  ctx.fillStyle = '#a78bfa'; ctx.beginPath(); ctx.arc(start[0], start[1], 5, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#f97316'; ctx.beginPath(); ctx.arc(end[0], end[1], 7, 0, Math.PI * 2); ctx.fill();
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
  document.getElementById('assessment-turn-translation').textContent = translationStatus === '—'
    ? '—'
    : `${translationStatus} · 候選 ${translationCandidate} · max XY ${translationMax.toFixed(3)} m · yaw ${translationYaw.toFixed(1)}°`;
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
    document.getElementById('rviz-status').textContent = s.rviz_status || 'RViz 尚未開啟；按鈕會自動開啟可操控的 macOS 螢幕共享';
    document.getElementById('r2-record-status').textContent = s.r2_recording_status || '尚未錄製建圖資料';
    document.getElementById('r2-download-status').textContent = s.r2_download_status || '輕量模式下載：CameraInfo、IMU、TF、odom、VO pose、SLAM path、landmarks cloud 與 GUI 摘要；完整模式才包含原始 IR 影像。';
    const p = s.odom_position || [0, 0, 0];
    document.getElementById('odom-x').textContent = `${p[0] >= 0 ? '+' : ''}${p[0].toFixed(3)} m`;
    document.getElementById('odom-y').textContent = `${p[1] >= 0 ? '+' : ''}${p[1].toFixed(3)} m`;
    document.getElementById('odom-z').textContent = `${p[2] >= 0 ? '+' : ''}${p[2].toFixed(3)} m`;
    document.getElementById('odom-yaw').textContent = `${s.odom_yaw_deg >= 0 ? '+' : ''}${s.odom_yaw_deg.toFixed(2)}°`;
    document.getElementById('odom-rate').textContent = `${s.odom_rate_hz.toFixed(1)} Hz`;
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
    updateAssessment(s.quality_assessment || {});
    drawOdomPath(s.odom_path || [], Number(s.run_start_yaw_deg || 0), Number(s.odom_yaw_deg || 0));
    document.querySelectorAll('[data-phase]').forEach(b => b.disabled = !s.rig_running || !!s.phase);
    document.getElementById('start').disabled = s.rig_running;
    document.getElementById('r2-start').disabled = s.r2_running || !!s.r2_busy;
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
        self.r2_diagnostics = "standard"
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
        self.rviz_status = "RViz 尚未開啟；按鈕會自動開啟可操控的 macOS 螢幕共享"
        self.rviz_opening = False
        self.rviz_tunnel_process: subprocess.Popen[str] | None = None
        self.rviz_viewer_process: subprocess.Popen[str] | None = None
        self.telemetry_process: subprocess.Popen[str] | None = None
        self.quality_process: subprocess.Popen[str] | None = None
        self.image_quality: dict[str, object] = {"connected": False, "flags": []}
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
        self.run_start_yaw_deg = 0.0
        self.run_yaw_delta_deg = 0.0
        self.run_yaw_unwrapped_deg = 0.0
        self.turn_history: list[dict[str, object]] = []
        self.odom_telemetry_count = 0
        self.odom_telemetry_valid_count = 0
        self.odom_path: list[list[float]] = []
        self.odom_path_length_m = 0.0
        self.odom_samples = 0
        self.odom_gap_ms = 0.0
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
            return {
                "status": self.status, "rig_running": self.rig_running, "phase": self.phase,
                "gyro": list(self.gyro), "peaks": list(self.peaks), "dominant": self.dominant,
                "elapsed": self.elapsed, "logs": list(self.logs), "results": list(self.results),
                "r2_status": self.r2_status, "r2_running": self.r2_running, "r2_mode": self.r2_mode,
                "r2_busy": self.r2_busy,
                "r2_mapping": self.r2_mapping,
                "r2_profile": self.r2_profile,
                "r2_lighting": self.r2_lighting,
                "r2_diagnostics": self.r2_diagnostics,
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
                "mapping_summary": self.mapping_summary_text(),
                "rviz_status": self.rviz_status,
                "rviz_opening": self.rviz_opening,
                "image_quality": dict(self.image_quality),
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
        jitter = self.turn_jitter_summary()
        turn_translation = self.turn_translation_summary()

        return "\n".join([
            "=== cuVSLAM R2 建圖分析摘要 ===",
            f"開始時間：{self.r2_started_at or '—'}",
            f"模式：{self.r2_mode} · 工作：{self.r2_mapping} · 影像：{self.r2_profile} · 光照：{self.r2_lighting}",
            f"特徵診斷：{self.r2_diagnostics}（features 才會輸出 observations cloud）",
            f"場景標籤：{self.r2_scene_label} · 動作標籤：{self.r2_motion_label}",
            f"品質判定：{assessment.get('label', '—')} · 分數：{score_text} / 100",
            f"VO 有效率：{vo_text} · 當前 vo_state：{self.odom_vo_state}",
            f"VO 統計來源：{vo_source_text}",
            f"odom rate：{self.odom_rate_hz:.1f} Hz · odom gap P95：{self.odom_gap_p95_ms:.1f} ms",
            f"odom rate 計算基準：{rate_text}",
            f"轉向抖動診斷：{jitter['status']} · turn samples={jitter['turn_samples']} · yaw rate P95={jitter['yaw_rate_p95_deg_s']:.1f}°/s · sign flips={jitter['sign_flips']}",
            f"整段 yaw Δ：{self.run_yaw_delta_deg:.1f}° · 累積 yaw：{self.run_yaw_unwrapped_deg:+.1f}°",
            f"最後轉向片段：{turn_translation['status']} · 原地旋轉候選={turn_translation['candidate_status']} · segments={turn_translation['turn_segments']} · 片段 yaw Δ={turn_translation['yaw_delta_deg']:.1f}° · max XY={turn_translation['max_xy_displacement_m']:.3f} m · turn path={turn_translation['path_length_m']:.3f} m",
            f"轉向片段完整紀錄：{len(self.turn_history)} 段",
            f"影像 rate L/R：{left_rate:.1f} / {right_rate:.1f} Hz · image gap P95 L/R：{left_gap:.1f} / {right_gap:.1f} ms",
            f"左右影像同步 P95：{stereo_sync_p95_us:.1f} µs（建議 ≤100 µs）",
            f"影像 clipped L/R：{left_clip:.1f}% / {right_clip:.1f}%",
            f"完整路徑點：{len(path)} · 累積路徑：{self.odom_path_length_m:.3f} m",
            f"目前位置：x={position[0]:+.3f} m, y={position[1]:+.3f} m, z={position[2]:+.3f} m",
            f"目前 yaw：{self.odom_yaw_deg:+.2f}° · 起點到目前 XY：{closure_xy:.3f} m",
            f"閉環參考：位置 ≤0.10 m Pass / ≤0.20 m Review；yaw ≤5° Pass / ≤10° Review",
            f"目前警告：{' · '.join(str(item) for item in warnings) if warnings else '無'}",
            "資料來源：/visual_slam/tracking/odometry、/visual_slam/status、RealSense IR image quality",
            f"完整追蹤紀錄：{self.run_trace_file or '—'} · records={self.run_trace_records}",
            f"本次 session：{self.run_session_file or '停止 R2 後產生'}",
            f"修正紀錄：{CORRECTION_LOG_PATH}",
        ])

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

    def quality_assessment(self) -> dict[str, object]:
        """Return a conservative, explainable online R2 quality assessment."""
        quality = self.image_quality
        left = quality.get("left", {}) if isinstance(quality, dict) else {}
        right = quality.get("right", {}) if isinstance(quality, dict) else {}
        left_window = left.get("window", {}) if isinstance(left, dict) else {}
        right_window = right.get("window", {}) if isinstance(right, dict) else {}
        stereo_sync = quality.get("stereo_sync", {}) if isinstance(quality, dict) else {}

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
                "vo_ratio_source": self.odom_vo_ratio_source,
                "vo_ratio_samples": self.odom_vo_ratio_samples,
                "turn_jitter": self.turn_jitter_summary(),
                "turn_translation": self.turn_translation_summary(),
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
        label = {"PASS": "PASS：可繼續測試", "REVIEW": "REVIEW：可測但需注意", "FAIL": "FAIL：先處理輸入或追蹤"}[overall]
        if self.odom_vo_ratio_source != "raw /visual_slam/status samples":
            warnings.append("VO 有效率統計尚非 raw status，分數僅供參考")
        if self.turn_jitter_status == "REVIEW":
            warnings.append("轉向抖動需注意")
        return {
            "overall": overall, "label": label, "score": round(score, 1),
            "vo_valid_ratio": vo_ratio, "odom_rate_hz": self.odom_rate_hz,
            "image_gap_p95_ms": image_gap, "clipping_pct": clipping,
            "stereo_sync_p95_us": stereo_sync_p95_us,
            "path_points": len(self.odom_path), "warnings": warnings,
            "vo_ratio_source": self.odom_vo_ratio_source,
            "vo_ratio_samples": self.odom_vo_ratio_samples,
            "turn_jitter": self.turn_jitter_summary(),
            "turn_translation": self.turn_translation_summary(),
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
            "turn_jitter": STATE.turn_jitter_summary(),
            "image_quality": dict(STATE.image_quality),
            "quality_assessment": STATE.quality_assessment(),
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
        STATE.run_start_yaw_deg = 0.0
        STATE.run_yaw_delta_deg = 0.0
        STATE.run_yaw_unwrapped_deg = 0.0
        STATE.turn_history = []
        STATE.odom_telemetry_count = 0
        STATE.odom_telemetry_valid_count = 0
        STATE.odom_path = []
        STATE.odom_path_length_m = 0.0
        STATE.odom_samples = 0
        STATE.odom_gap_ms = 0.0
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
            ssh_command(["bash", REMOTE_RVIZ_VIEW_SCRIPT, "open"]),
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


def handle_telemetry_line(line: str) -> None:
    if not line.startswith("LIVE_ODOM "):
        if line:
            STATE.add_log(line)
        return
    try:
        data = json.loads(line[len("LIVE_ODOM "):])
        with STATE.lock:
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
                if STATE.turn_translation_candidate_status in {"REVIEW", "FAIL"} and previous_translation_status != STATE.turn_translation_status:
                    STATE.add_log(
                        "旋轉期間估計平移候選進入 "
                        f"{STATE.turn_translation_candidate_status}（待確認動作類型）："
                        f"yaw_delta={STATE.turn_translation_yaw_delta_deg:.1f} deg "
                        f"max_xy={STATE.turn_translation_max_xy_m:.3f} m"
                    )
            if data.get("vo_valid_ratio") is not None:
                STATE.odom_vo_valid_ratio = float(data["vo_valid_ratio"])
                STATE.odom_vo_ratio_source = "raw /visual_slam/status samples"
                STATE.odom_vo_ratio_samples = int(data.get("vo_status_samples", 0))
                STATE.odom_vo_valid_samples = int(data.get("vo_valid_samples", 0))
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
            STATE.odom_path_length_m = float(data.get("path_length_m", 0.0))
            STATE.odom_samples = int(data.get("samples", 0))
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
            STATE.r2_status = "R2 odom 即時連線中"
        trace_data = dict(data)
        trace_data.pop("path", None)
        trace_data["path_point_count"] = len(data.get("path", [])) if isinstance(data.get("path", []), list) else 0
        append_run_trace("odom_telemetry", trace_data)
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        STATE.add_log(f"odom telemetry parse failed: {exc}")


def telemetry_worker(process: subprocess.Popen[str]) -> None:
    assert process.stdout is not None
    for raw_line in process.stdout:
        handle_telemetry_line(raw_line.strip())
    process.wait()
    with STATE.lock:
        if STATE.telemetry_process is process:
            STATE.telemetry_process = None
        if STATE.r2_running:
            STATE.r2_status = "R2 已啟動，但 odom telemetry 已中斷"


def handle_quality_line(line: str) -> None:
    if not line.startswith("LIVE_QUALITY "):
        if line:
            STATE.add_log(line)
        return
    try:
        data = json.loads(line[len("LIVE_QUALITY "):])
        with STATE.lock:
            data["connected"] = True
            STATE.image_quality = data
            if STATE.motion_recording:
                STATE.motion_quality_samples.append(
                    {
                        "elapsed_s": time.monotonic() - STATE.motion_started_at,
                        "flags": list(data.get("flags", [])),
                        "left": data.get("left", {}).get("window", {}),
                        "right": data.get("right", {}).get("window", {}),
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


def start_telemetry() -> None:
    with STATE.lock:
        if STATE.telemetry_process is not None and STATE.telemetry_process.poll() is None:
            return
    process = subprocess.Popen(
        ssh_command(["bash", REMOTE_R2_SCRIPT, "telemetry"]),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    with STATE.lock:
        STATE.telemetry_process = process
    threading.Thread(target=telemetry_worker, args=(process,), daemon=True).start()


def start_quality() -> None:
    with STATE.lock:
        if STATE.quality_process is not None and STATE.quality_process.poll() is None:
            return
    process = subprocess.Popen(
        ssh_command(["bash", REMOTE_R2_SCRIPT, "quality"]),
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
    scene: str, motion: str, diagnostics: str,
) -> None:
    try:
        result = subprocess.run(
            ssh_command([
                "bash", REMOTE_R2_SCRIPT, "start", mode, mapping, "no_reset",
                "slam", profile, lighting, diagnostics,
            ]),
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
            STATE.r2_diagnostics = diagnostics
            STATE.r2_scene_label = scene
            STATE.r2_motion_label = motion
            STATE.r2_status = "R2 容器已啟動，正在連接 odom…" if result.returncode == 0 else "R2 啟動失敗"
        if result.returncode == 0:
            append_run_trace("r2_remote_started", {
                "scene": scene, "motion": motion, "diagnostics": diagnostics,
            })
            start_telemetry()
            start_quality()
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(str(exc))
        with STATE.lock:
            STATE.r2_running = False
            STATE.r2_busy = False
            STATE.r2_status = "R2 啟動失敗"


def start_r2(
    mode: str, mapping: str, profile: str, lighting: str,
    scene: str = "scene_unknown", motion: str = "other",
    diagnostics: str = "standard",
) -> str:
    if mode not in {"stereo", "imu"}:
        return "R2 模式無效"
    if mapping not in {"odom", "mapping"}:
        return "R2 工作模式無效"
    if profile not in {"640x360x30", "848x480x30"}:
        return "R2 影像 profile 無效"
    if lighting not in {"official", "baseline", "robust", "low_light", "high_contrast"}:
        return "R2 光照策略無效"
    if diagnostics not in {"standard", "features"}:
        return "R2 特徵診斷模式無效"
    scene = " ".join(scene.split())[:48] or "scene_unknown"
    if motion not in {"forward_left_turn", "forward_right_turn", "pure_turn", "straight", "closed_loop", "other"}:
        motion = "other"
    with STATE.lock:
        if STATE.r2_running or STATE.r2_busy:
            return "R2 容器已在執行"
        STATE.r2_busy = True
        STATE.r2_mode = mode
        STATE.r2_mapping = mapping
        STATE.r2_profile = profile
        STATE.r2_lighting = lighting
        STATE.r2_diagnostics = diagnostics
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
            f"scene={scene} motion={motion}"
        )
        STATE.r2_status = "正在啟動 R2 cuVSLAM…"
    reset_r2_live_state()
    append_run_trace("r2_start_requested", {
        "mode": mode, "mapping": mapping, "profile": profile, "lighting": lighting,
        "scene": scene, "motion": motion, "diagnostics": diagnostics,
    })
    threading.Thread(
        target=r2_start_worker,
        args=(mode, mapping, profile, lighting, scene, motion, diagnostics),
        daemon=True,
    ).start()
    return "正在啟動 R2 cuVSLAM…"


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
            ssh_command(["bash", REMOTE_R2_SCRIPT, "record_stop", run_name]),
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


def r2_record_start_worker(run_name: str, record_mode: str) -> None:
    try:
        result = subprocess.run(
            ssh_command(["bash", REMOTE_R2_SCRIPT, "record_start", run_name, record_mode]),
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
                STATE.r2_recording_status = "建圖錄製啟動失敗"
    except Exception as exc:  # noqa: BLE001
        STATE.add_log(f"建圖錄製啟動失敗：{exc}")
        with STATE.lock:
            STATE.r2_recording_status = "建圖錄製啟動失敗"


def start_r2_recording(record_mode: str) -> str:
    if record_mode not in {"light", "full"}:
        return "錄製模式無效"
    with STATE.lock:
        if not STATE.r2_running:
            return "請先啟動 R2"
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
    remote = f"{USER}@{HOST}:{REMOTE_R2_DATA_DIR}/{run_name}"
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
            return "尚未有可下載的建圖錄製；下一次 R2 請先按開始錄製建圖資料"
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
            ssh_command(["bash", REMOTE_R2_SCRIPT, "cleanup"]),
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
            ssh_command(["bash", REMOTE_RVIZ_VIEW_SCRIPT, "reset"]),
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
            STATE.r2_status = "R2 已停止；R2 與 RViz 畫面都已清除"
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
            return "請先啟動 R2 odom"
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


def run_simple(args: list[str], success: str, failure: str) -> None:
    try:
        result = subprocess.run(ssh_command(args), capture_output=True, text=True, timeout=100)
        output = (result.stdout + result.stderr).strip()
        if output:
            for line in output.splitlines()[-30:]:
                STATE.add_log(line)
        with STATE.lock:
            STATE.status = success if result.returncode == 0 else failure
            if args[-1] == "start":
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
    threading.Thread(target=run_simple, args=(["bash", REMOTE_SCRIPT, "start"], "實機容器已啟動，可開始 phase", "啟動失敗"), daemon=True).start()
    return "正在啟動 Jetson D435i + cuVSLAM…"


def stop_rig() -> str:
    process = STATE.phase_process
    if process is not None and process.poll() is None:
        process.terminate()
    with STATE.lock:
        STATE.phase = ""; STATE.phase_process = None; STATE.status = "正在停止實機容器…"; STATE.rig_running = False
    threading.Thread(target=run_simple, args=(["bash", REMOTE_SCRIPT, "stop"], "實機容器已停止", "停止完成或容器不存在"), daemon=True).start()
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
    process = subprocess.Popen(ssh_command(["bash", REMOTE_SCRIPT, "phase", phase, f"{duration:g}", output_name]), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
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
            elif path == "/api/r2/start": self.send_json({"message": start_r2(str(data.get("mode", "stereo")), str(data.get("mapping", "odom")), str(data.get("profile", "640x360x30")), str(data.get("lighting", "robust")), str(data.get("scene", "scene_unknown")), str(data.get("motion", "other")), str(data.get("diagnostics", "standard")))})
            elif path == "/api/r2/stop": self.send_json({"message": stop_r2()})
            elif path == "/api/r2/record/start": self.send_json({"message": start_r2_recording(str(data.get("mode", "light")))})
            elif path == "/api/r2/record/stop": self.send_json({"message": stop_r2_recording()})
            elif path == "/api/r2/record/download": self.send_json({"message": download_r2_recording()})
            elif path == "/api/rviz/open": self.send_json({"message": open_rviz()})
            elif path == "/api/r2/motion/start": self.send_json({"message": start_motion_record()})
            elif path == "/api/r2/motion/stop": self.send_json({"message": stop_motion_record()})
            elif path == "/api/r2/motion/clear": self.send_json({"message": clear_motion_record()})
            elif path == "/api/phase":
                phase = str(data.get("phase", "")); duration = float(data.get("duration", 10))
                if phase not in PHASES or not 3 <= duration <= 60: self.send_json({"message": "phase 或秒數無效"}, 400)
                else: self.send_json({"message": start_phase(phase, duration)})
            else: self.send_json({"error": "not found"}, 404)
        except Exception as exc:  # noqa: BLE001
            self.send_json({"message": str(exc)}, 500)


def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("--port", type=int, default=8765); parser.add_argument("--open-browser", action="store_true"); args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler); url = f"http://127.0.0.1:{args.port}/"
    print(f"Real D435i GUI: {url}", flush=True)
    if args.open_browser: threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        stop_rig()
        stop_r2(wait=True)
        server.server_close()


if __name__ == "__main__": main()
