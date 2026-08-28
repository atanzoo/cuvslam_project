#!/usr/bin/env python3
"""Local web GUI for controlling the Jetson D435i simulation."""

from __future__ import annotations

import json
import atexit
import os
import signal
import socket
import subprocess
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from jetson_connection_gui import JetsonConnectionApp


HOST = "127.0.0.1"
PORT = 8765
MAX_LOG_LINES = 1000
ISAAC_CONTAINER = "isaac_ros_dev-aarch64-container"
VSLAM_LAUNCH = "isaac_ros_visual_slam_d435i_sim.launch.py"
VSLAM_LOG = "/tmp/cuvslam-d435i-vslam.log"
VSLAM_CAMERA_X = "0.19"
VSLAM_WORLD_HOST = (
    "/home/tseng/jetson_slam_ws/install/slam_gazebo/share/slam_gazebo/worlds/"
    "cuvslam_mapping_observable_640x360_range2p5_dense_turn.sdf"
)
VSLAM_WORLD_CONTAINER = "/workspaces/isaac_ros-dev/data/gui_active_world.sdf"
VSLAM_NATIVE_POSE_TOPIC = (
    "/world/cuvslam_mapping_simple/dynamic_pose/info"
)
VSLAM_ENABLE_MAPPING = "true"
VSLAM_ENABLE_IMU_FUSION = "false"
VSLAM_ENABLE_SLAM_VISUALIZATION = "true"
VSLAM_ENABLE_OBSERVATIONS_VIEW = "true"
MONITOR_INTERVAL_SECONDS = 15
FOXGLOVE_REMOTE_PORT = 8765
FOXGLOVE_LOCAL_PORT = 8766
MAPPING_ROUTE_SCRIPT = (
    "/workspaces/isaac_ros-dev/tools/run_container_straight_mapping_trial.sh"
)
FOXGLOVE_APP = "/Applications/Foxglove.app"
FOXGLOVE_LAUNCHER = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "Foxglove Simulation.command",
)
NETWORKSETUP = "/usr/sbin/networksetup"
L4T_NETWORK_SERVICE = "Linux for Tegra"
L4T_HOST_ADDRESS = "192.168.55.100"
L4T_JETSON_ADDRESS = "192.168.55.1"
MONITOR_UNITS = {
    "baseline_gazebo": "cuvslam-d435i-gazebo.service",
    "baseline_bridge": "cuvslam-d435i-bridge.service",
    "test_gazebo": "cuvslam-mapping-simple-gazebo.service",
    "test_bridge": "cuvslam-mapping-simple-bridge.service",
}
MONITOR_TOPICS = {
    "infra1_info": "/d435i/infra1/camera_info",
    "infra1_image": "/d435i/infra1/image_rect_raw",
    "infra2_info": "/d435i/infra2/camera_info",
    "infra2_image": "/d435i/infra2/image_rect_raw",
    "imu": "/d435i/imu",
    "scan": "/scan",
    "truth": "/simulation/native_pose_raw",
    "clock": "/clock",
    "world_outline": "/simulation/ground_truth_world_markers",
}

PAGE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>cuVSLAM Jetson Control</title>
  <style>
    :root {
      color-scheme: light;
      font-family: Inter, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #ffffff;
      color: #111827;
    }
    * { box-sizing: border-box; letter-spacing: 0; }
    body { margin: 0; min-width: 780px; min-height: 560px; background: #ffffff; }
    button, input { font: inherit; }
    button { cursor: pointer; }
    button:disabled { cursor: wait; opacity: 0.5; }
    .app {
      display: grid;
      grid-template-columns: minmax(360px, 430px) minmax(420px, 1fr);
      width: 100vw;
      height: 100vh;
      min-height: 560px;
      overflow: hidden;
      background: #ffffff;
    }
    .controls {
      display: flex;
      flex-direction: column;
      min-width: 0;
      padding: 28px 30px;
      background: #ffffff;
      border-right: 1px solid #d8dee8;
      overflow-y: auto;
    }
    .brand { margin: 0 0 4px; font-size: 24px; line-height: 1.2; font-weight: 750; }
    .subtitle { margin: 0 0 28px; color: #64748b; font-size: 14px; }
    .section {
      width: 100%;
      padding: 22px 0;
      border-top: 1px solid #e5e7eb;
    }
    .section:first-of-type { padding-top: 0; border-top: 0; }
    .section h2 { margin: 0 0 14px; font-size: 15px; line-height: 1.3; }
    .field { display: grid; grid-template-columns: 86px minmax(0, 1fr); align-items: center; gap: 12px; margin: 10px 0; }
    .field label { color: #334155; font-size: 13px; font-weight: 650; }
    .field input {
      width: 100%;
      height: 38px;
      padding: 0 10px;
      color: #111827;
      background: #ffffff;
      border: 1px solid #bcc5d2;
      border-radius: 5px;
      outline: none;
    }
    .field input:focus { border-color: #1473e6; box-shadow: 0 0 0 2px #dbeafe; }
    .button-row { display: grid; gap: 9px; width: 100%; margin-top: 15px; }
    .button-row.two { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .button-row.three { grid-template-columns: repeat(3, minmax(0, 1fr)); }
    .button {
      height: 38px;
      padding: 0 12px;
      border: 1px solid #b9c2ce;
      border-radius: 5px;
      color: #182231;
      background: #f8fafc;
      font-size: 13px;
      font-weight: 680;
      white-space: nowrap;
    }
    .button:hover:not(:disabled) { background: #edf1f5; }
    .button.primary { color: #ffffff; border-color: #1266c5; background: #1473e6; }
    .button.primary:hover:not(:disabled) { background: #0f62bd; }
    .button.danger { color: #b42318; border-color: #e5aaa4; background: #fffafa; }
    .monitor-heading { display: flex; align-items: center; gap: 10px; }
    .monitor-heading h2 { margin-right: auto; }
    .monitor-badge {
      display: inline-flex;
      align-items: center;
      min-height: 24px;
      padding: 0 8px;
      border: 1px solid #cbd5e1;
      border-radius: 4px;
      color: #475569;
      background: #f8fafc;
      font-size: 11px;
      font-weight: 700;
    }
    .monitor-badge.ok { color: #166534; border-color: #86c99a; background: #f0fdf4; }
    .monitor-badge.warn { color: #92400e; border-color: #e8c478; background: #fffbeb; }
    .monitor-badge.error { color: #991b1b; border-color: #e5a09a; background: #fef2f2; }
    .monitor-grid {
      display: grid;
      grid-template-columns: 94px minmax(0, 1fr);
      border-top: 1px solid #e5e7eb;
      font-size: 12px;
      line-height: 1.35;
    }
    .monitor-label, .monitor-value {
      min-width: 0;
      padding: 7px 0;
      border-bottom: 1px solid #e5e7eb;
    }
    .monitor-label { color: #64748b; }
    .monitor-value {
      color: #1f2937;
      font-weight: 650;
      overflow-wrap: anywhere;
    }
    .status {
      margin-top: auto;
      padding-top: 20px;
      border-top: 1px solid #e5e7eb;
    }
    .status-line { display: flex; align-items: flex-start; gap: 9px; color: #475569; font-size: 13px; line-height: 1.5; }
    .status-dot { flex: 0 0 auto; width: 9px; height: 9px; margin-top: 5px; border-radius: 50%; background: #94a3b8; }
    .status-dot.busy { background: #d97706; }
    .status-dot.ok { background: #16803b; }
    .status-dot.error { background: #c9362b; }
    .log-panel {
      display: grid;
      grid-template-rows: 65px minmax(0, 1fr);
      min-width: 0;
      min-height: 0;
      background: #0c111b;
      color: #e5e7eb;
    }
    .log-toolbar {
      display: flex;
      align-items: center;
      gap: 8px;
      padding: 0 18px;
      border-bottom: 1px solid #263143;
    }
    .log-toolbar h2 { margin: 0 auto 0 0; font-size: 15px; color: #f8fafc; }
    .log-button {
      height: 32px;
      padding: 0 12px;
      border: 1px solid #4a586d;
      border-radius: 5px;
      color: #e5e7eb;
      background: #202b3b;
      font-size: 12px;
      font-weight: 650;
    }
    .log-button:hover { background: #2a374a; }
    .log-body {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 18px;
      min-width: 0;
      min-height: 0;
      background: #0c111b;
    }
    #log {
      min-width: 0;
      min-height: 0;
      margin: 0;
      padding: 18px 20px;
      overflow-y: scroll;
      overscroll-behavior: contain;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      color: #d7dee9;
      background: #0c111b;
      font: 12px/1.6 Menlo, Monaco, Consolas, monospace;
      scrollbar-width: none;
    }
    #log::-webkit-scrollbar { display: none; }
    #log-scroll {
      width: 18px;
      height: 100%;
      margin: 0;
      padding: 0;
      writing-mode: vertical-lr;
      direction: rtl;
      appearance: slider-vertical;
      -webkit-appearance: slider-vertical;
      accent-color: #74859e;
      background: #151d29;
      cursor: ns-resize;
    }
    #log-scroll:disabled { opacity: 0.6; cursor: default; }
    @media (max-width: 900px) {
      body { min-width: 0; }
      .app { grid-template-columns: 1fr; grid-template-rows: auto minmax(360px, 1fr); height: auto; min-height: 100vh; overflow: visible; }
      .controls { border-right: 0; border-bottom: 1px solid #d8dee8; }
      .log-panel { min-height: 460px; }
    }
  </style>
</head>
<body>
  <main class="app">
    <section class="controls">
      <h1 class="brand">cuVSLAM Jetson Control</h1>
      <p class="subtitle">D435i simulation operator panel</p>

      <div class="section">
        <h2>Connection</h2>
        <div class="field"><label for="hosts">Host(s)</label><input id="hosts" value="192.168.55.1,192.168.2.2"></div>
        <div class="field"><label for="user">User</label><input id="user" value="tseng"></div>
        <div class="field"><label for="port">SSH port</label><input id="port" value="22" inputmode="numeric"></div>
        <div class="button-row two">
          <button class="button action" data-action="check">Check connection</button>
          <button class="button action" data-action="reconnect">Reconnect SSH</button>
        </div>
      </div>

      <div class="section">
        <div class="monitor-heading">
          <h2>Simulation monitor</h2>
          <span id="monitor-badge" class="monitor-badge">Stopped</span>
        </div>
        <div class="monitor-grid" aria-live="polite">
          <span class="monitor-label">World</span><span id="monitor-world" class="monitor-value">Not sampled</span>
          <span class="monitor-label">Gazebo</span><span id="monitor-gazebo" class="monitor-value">-</span>
          <span class="monitor-label">Bridge</span><span id="monitor-bridge" class="monitor-value">-</span>
          <span class="monitor-label">D435i</span><span id="monitor-d435i" class="monitor-value">-</span>
          <span class="monitor-label">LiDAR / native</span><span id="monitor-extra-topics" class="monitor-value">-</span>
          <span class="monitor-label">World outline</span><span id="monitor-world-outline" class="monitor-value">-</span>
          <span class="monitor-label">cuVSLAM</span><span id="monitor-vslam" class="monitor-value">-</span>
          <span class="monitor-label">Endpoint</span><span id="monitor-endpoint" class="monitor-value">-</span>
          <span class="monitor-label">Updated</span><span id="monitor-updated" class="monitor-value">-</span>
        </div>
        <div class="button-row two">
          <button id="monitor-start" class="button primary">Start monitor</button>
          <button id="monitor-stop" class="button danger" disabled>Stop monitor</button>
        </div>
      </div>

      <div class="section">
        <h2>Simulation</h2>
        <p class="subtitle">Dense v3 world · stereo-only cuVSLAM · initial corner mapping · Foxglove viewer</p>
        <div class="button-row">
          <button class="button primary action" data-action="start">Start simulation</button>
        </div>
        <div class="button-row two">
          <button class="button danger action" data-action="stop">Stop</button>
          <button class="button action" data-action="status">Refresh status</button>
        </div>
      </div>

      <div class="status">
        <div class="status-line">
          <span id="status-dot" class="status-dot"></span>
          <span id="status-text">Ready</span>
        </div>
      </div>
    </section>

    <section class="log-panel">
      <header class="log-toolbar">
        <h2>Work log</h2>
        <button id="copy-log" class="log-button">Copy log</button>
        <button id="clear-log" class="log-button">Clear log</button>
      </header>
      <div class="log-body">
        <pre id="log" aria-live="polite"></pre>
        <input id="log-scroll" type="range" min="0" max="1" value="0" aria-label="Work log scroll position">
      </div>
    </section>
  </main>

  <script>
    const logNode = document.querySelector("#log");
    const logScroll = document.querySelector("#log-scroll");
    const statusText = document.querySelector("#status-text");
    const statusDot = document.querySelector("#status-dot");
    const actionButtons = [...document.querySelectorAll(".action")];
    const monitorStart = document.querySelector("#monitor-start");
    const monitorStop = document.querySelector("#monitor-stop");
    const monitorBadge = document.querySelector("#monitor-badge");
    let renderedLog = "";

    function syncLogScroll() {
      const maximum = Math.max(0, logNode.scrollHeight - logNode.clientHeight);
      logScroll.max = String(Math.max(1, maximum));
      logScroll.value = String(logNode.scrollTop);
      logScroll.disabled = maximum === 0;
    }

    function settings() {
      return {
        hosts: document.querySelector("#hosts").value,
        user: document.querySelector("#user").value,
        port: document.querySelector("#port").value
      };
    }

    async function request(path, options = {}) {
      const response = await fetch(path, options);
      const data = await response.json();
      if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
      return data;
    }

    function render(snapshot) {
      const nextLog = snapshot.logs.join("\n");
      const follow = logNode.scrollHeight - logNode.scrollTop - logNode.clientHeight < 48;
      if (nextLog !== renderedLog) {
        logNode.textContent = nextLog;
        renderedLog = nextLog;
        if (follow || snapshot.busy) logNode.scrollTop = logNode.scrollHeight;
      }
      requestAnimationFrame(syncLogScroll);
      statusText.textContent = snapshot.status;
      statusDot.className = `status-dot ${snapshot.level || ""}`;
      actionButtons.forEach(button => { button.disabled = snapshot.busy; });
      const monitor = snapshot.monitor || {};
      monitorBadge.textContent = monitor.enabled ? (monitor.state || "Sampling") : "Stopped";
      monitorBadge.className = `monitor-badge ${monitor.enabled ? (monitor.level || "") : ""}`;
      document.querySelector("#monitor-world").textContent = monitor.world || "Not sampled";
      document.querySelector("#monitor-gazebo").textContent = monitor.gazebo || "-";
      document.querySelector("#monitor-bridge").textContent = monitor.bridge || "-";
      document.querySelector("#monitor-d435i").textContent = monitor.d435i || "-";
      document.querySelector("#monitor-extra-topics").textContent = monitor.extra_topics || "-";
      document.querySelector("#monitor-world-outline").textContent = monitor.world_outline || "-";
      document.querySelector("#monitor-vslam").textContent = monitor.vslam || "-";
      document.querySelector("#monitor-endpoint").textContent = monitor.endpoint || "-";
      document.querySelector("#monitor-updated").textContent = monitor.updated || "-";
      monitorStart.disabled = snapshot.busy || Boolean(monitor.enabled);
      monitorStop.disabled = !monitor.enabled;
    }

    async function refresh() {
      try {
        render(await request("/api/snapshot"));
      } catch (error) {
        statusText.textContent = `GUI server unavailable: ${error.message}`;
        statusDot.className = "status-dot error";
      }
    }

    actionButtons.forEach(button => {
      button.addEventListener("click", async () => {
        statusText.textContent = `${button.textContent.trim()} requested...`;
        statusDot.className = "status-dot busy";
        try {
          render(await request("/api/action", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify({action: button.dataset.action, ...settings()})
          }));
        } catch (error) {
          statusText.textContent = error.message;
          statusDot.className = "status-dot error";
        }
      });
    });

    async function setMonitor(enabled) {
      statusText.textContent = `${enabled ? "Start" : "Stop"} monitor requested...`;
      statusDot.className = "status-dot busy";
      try {
        render(await request(`/api/monitor/${enabled ? "start" : "stop"}`, {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify(settings())
        }));
      } catch (error) {
        statusText.textContent = error.message;
        statusDot.className = "status-dot error";
      }
    }

    monitorStart.addEventListener("click", () => setMonitor(true));
    monitorStop.addEventListener("click", () => setMonitor(false));

    document.querySelector("#copy-log").addEventListener("click", async () => {
      try {
        await navigator.clipboard.writeText(logNode.textContent);
        statusText.textContent = "Work log copied to clipboard";
      } catch (error) {
        statusText.textContent = `Copy failed: ${error.message}`;
        statusDot.className = "status-dot error";
      }
    });

    document.querySelector("#clear-log").addEventListener("click", async () => {
      try {
        render(await request("/api/logs/clear", {method: "POST"}));
      } catch (error) {
        statusText.textContent = `Clear failed: ${error.message}`;
        statusDot.className = "status-dot error";
      }
    });

    logNode.addEventListener("scroll", syncLogScroll);
    logScroll.addEventListener("input", () => {
      logNode.scrollTop = Number(logScroll.value);
    });

    let cleanupSent = false;
    window.addEventListener("pagehide", () => {
      if (cleanupSent) return;
      cleanupSent = true;
      const payload = new Blob([JSON.stringify(settings())], {
        type: "application/json"
      });
      navigator.sendBeacon("/api/cleanup", payload);
    });

    refresh();
    setInterval(refresh, 500);
  </script>
</body>
</html>
"""


class Controller:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.remote_lock = threading.Lock()
        self.logs: list[str] = []
        self.busy = False
        self.status = "Ready"
        self.level = ""
        self.pending_cleanup: tuple[list[tuple[str, int]], str] | None = None
        self.cleanup_in_progress = False
        self.monitor_stop_event: threading.Event | None = None
        self.monitor = self.empty_monitor()
        self.backend = object.__new__(JetsonConnectionApp)
        self.backend.log_message = self.log
        self.log("GUI server ready on localhost.")

    @staticmethod
    def empty_monitor() -> dict:
        return {
            "enabled": False,
            "state": "Stopped",
            "level": "",
            "world": "Not sampled",
            "gazebo": "-",
            "bridge": "-",
            "d435i": "-",
            "extra_topics": "-",
            "world_outline": "-",
            "vslam": "-",
            "endpoint": "-",
            "updated": "-",
        }

    def log(self, message: str, level: str = "INFO") -> None:
        timestamp = time.strftime("%H:%M:%S")
        lines = message.rstrip().splitlines() or [""]
        with self.lock:
            for line in lines:
                self.logs.append(f"[{timestamp}] [{level}] {line}")
            if len(self.logs) > MAX_LOG_LINES:
                self.logs = self.logs[-MAX_LOG_LINES:]

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "logs": list(self.logs),
                "busy": self.busy,
                "status": self.status,
                "level": self.level,
                "monitor": dict(self.monitor),
            }

    def clear_logs(self) -> dict:
        with self.lock:
            self.logs.clear()
        return self.snapshot()

    @staticmethod
    def settings(payload: dict) -> tuple[list[tuple[str, int]], str]:
        hosts = [item.strip() for item in str(payload.get("hosts", "")).split(",") if item.strip()]
        user = str(payload.get("user", "")).strip()
        if not hosts:
            raise ValueError("At least one host is required")
        if not user:
            raise ValueError("User is required")
        try:
            port = int(str(payload.get("port", "")).strip())
        except ValueError as exc:
            raise ValueError("SSH port must be a number") from exc
        if not 1 <= port <= 65535:
            raise ValueError("SSH port must be between 1 and 65535")
        return [(host, port) for host in hosts], user

    def find_endpoint(self, endpoints: list[tuple[str, int]]) -> tuple[str, int] | None:
        for host, port in endpoints:
            self.log(f"Probing SSH endpoint {host}:{port}.")
            try:
                with socket.create_connection((host, port), timeout=2):
                    return host, port
            except OSError as exc:
                self.log(f"{host}:{port} unavailable: {exc}", "WARN")
        return None

    def recover_l4t_usb_network(
        self,
        endpoints: list[tuple[str, int]],
    ) -> bool:
        if not any(host == L4T_JETSON_ADDRESS for host, _ in endpoints):
            return False
        info = subprocess.run(
            [NETWORKSETUP, "-getinfo", L4T_NETWORK_SERVICE],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if (
            info.returncode
            or f"IP address: {L4T_HOST_ADDRESS}" not in info.stdout
        ):
            self.log(
                "Linux for Tegra does not have the expected "
                f"{L4T_HOST_ADDRESS} static profile; automatic recovery skipped.",
                "WARN",
            )
            return False

        self.log(
            "Resetting the Linux for Tegra network service to clear a stale "
            "Internet Sharing bridge.",
            "WARN",
        )
        for enabled in ("off", "on"):
            result = subprocess.run(
                [
                    NETWORKSETUP,
                    "-setnetworkserviceenabled",
                    L4T_NETWORK_SERVICE,
                    enabled,
                ],
                capture_output=True,
                text=True,
                timeout=8,
            )
            if result.returncode:
                detail = result.stderr.strip() or result.stdout.strip()
                self.log(
                    f"Linux for Tegra reset failed while switching {enabled}: "
                    f"{detail or result.returncode}",
                    "WARN",
                )
                return False
        time.sleep(1.0)
        self.log(
            f"Linux for Tegra restored; retrying {L4T_JETSON_ADDRESS}.",
            "OK",
        )
        return True

    @staticmethod
    def find_endpoint_quiet(
        endpoints: list[tuple[str, int]],
    ) -> tuple[str, int] | None:
        for host, port in endpoints:
            try:
                with socket.create_connection((host, port), timeout=2):
                    return host, port
            except OSError:
                continue
        return None

    def start_monitor(self, payload: dict) -> dict:
        endpoints, user = self.settings(payload)
        return self.enable_monitor(endpoints, user)

    def enable_monitor(
        self,
        endpoints: list[tuple[str, int]],
        user: str,
    ) -> dict:
        with self.lock:
            if self.monitor["enabled"]:
                return self.snapshot_unlocked()
            stop_event = threading.Event()
            self.monitor_stop_event = stop_event
            self.monitor = {
                **self.empty_monitor(),
                "enabled": True,
                "state": "Starting",
                "level": "warn",
            }
            self.status = (
                f"Simulation monitor started; sampling every "
                f"{MONITOR_INTERVAL_SECONDS}s"
            )
            self.level = "ok"
        endpoint_text = ", ".join(f"{host}:{port}" for host, port in endpoints)
        self.log(
            f"SSH low-rate monitor started. Candidates: {endpoint_text}; "
            f"interval={MONITOR_INTERVAL_SECONDS}s.",
            "OK",
        )
        threading.Thread(
            target=self._monitor_loop,
            args=(endpoints, user, stop_event),
            daemon=True,
        ).start()
        return self.snapshot()

    def stop_monitor(self) -> dict:
        with self.lock:
            stop_event = self.monitor_stop_event
            was_enabled = self.monitor["enabled"]
            self.monitor_stop_event = None
            self.monitor = self.empty_monitor()
            self.status = "Simulation monitor stopped"
            self.level = ""
        if stop_event is not None:
            stop_event.set()
        if was_enabled:
            self.log("SSH low-rate monitor stopped.", "OK")
        return self.snapshot()

    def snapshot_unlocked(self) -> dict:
        return {
            "logs": list(self.logs),
            "busy": self.busy,
            "status": self.status,
            "level": self.level,
            "monitor": dict(self.monitor),
        }

    @staticmethod
    def monitor_shell() -> str:
        lines = [
            "set +e",
            "unit_state() {",
            '  state="$(systemctl --user is-active "$2" 2>/dev/null)"',
            '  [ -n "$state" ] || state="unknown"',
            "  printf '%s=%s\\n' \"$1\" \"$state\"",
            "}",
        ]
        for key, unit in MONITOR_UNITS.items():
            lines.append(f"unit_state {key} {unit}")
        lines.extend(
            [
                "source /opt/ros/foxy/setup.bash",
                "export ROS_DOMAIN_ID=43",
                "export ROS2CLI_DISABLE_DAEMON=1",
                'topics="$(timeout 6 ros2 topic list 2>/dev/null)"',
                "topic_flag() {",
                '  if grep -Fqx "$2" <<< "$topics"; then value=ready; else value=missing; fi',
                "  printf '%s=%s\\n' \"$1\" \"$value\"",
                "}",
            ]
        )
        for key, topic in MONITOR_TOPICS.items():
            lines.append(f"topic_flag topic_{key} {topic}")
        vslam_status_shell = (
            "source /opt/ros/humble/setup.bash; "
            "source /workspaces/isaac_ros-dev/install/setup.bash; "
            "export ROS_DOMAIN_ID=43; "
            "export ROS2CLI_DISABLE_DAEMON=1; "
            "timeout 6 ros2 topic echo --once /visual_slam/status"
        )
        lines.extend(
            [
                (
                    f"if docker exec {ISAAC_CONTAINER} pgrep -f {VSLAM_LAUNCH} "
                    ">/dev/null 2>&1; then"
                ),
                "  printf 'vslam_process=running\\n'",
                (
                    f"  status=\"$(docker exec {ISAAC_CONTAINER} bash -lc "
                    f"'{vslam_status_shell}' 2>/dev/null)\""
                ),
                (
                    "  vo_state=\"$(printf '%s\\n' \"$status\" | "
                    "sed -n 's/^[[:space:]]*vo_state:[[:space:]]*//p' | head -n 1)\""
                ),
                '  [ -n "$vo_state" ] || vo_state="unknown"',
                "  printf 'vo_state=%s\\n' \"$vo_state\"",
                "else",
                "  printf 'vslam_process=stopped\\n'",
                "  printf 'vo_state=none\\n'",
                "fi",
            ]
        )
        return "\n".join(lines)

    def collect_monitor_sample(
        self,
        target: str,
        port: int,
        host: str,
    ) -> tuple[dict, tuple]:
        with self.remote_lock:
            result = self.backend.run_ssh(
                target,
                port,
                ["/bin/bash", "-lc", self.monitor_shell()],
                timeout=24,
                check=False,
            )
        if result.returncode:
            detail = result.stderr.strip() or f"SSH command exit {result.returncode}"
            raise RuntimeError(detail)

        values = {}
        for line in result.stdout.splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                values[key.strip()] = value.strip()

        baseline_states = (
            values.get("baseline_gazebo", "unknown"),
            values.get("baseline_bridge", "unknown"),
        )
        test_states = (
            values.get("test_gazebo", "unknown"),
            values.get("test_bridge", "unknown"),
        )
        baseline_any = "active" in baseline_states
        test_any = "active" in test_states
        if baseline_any and test_any:
            world = "CONFLICT: baseline + test"
            world_level = "error"
        elif baseline_states == ("active", "active"):
            world = "Baseline"
            world_level = ""
        elif test_states == ("active", "active"):
            world = "Mapping simple"
            world_level = ""
        elif not baseline_any and not test_any:
            world = "Stopped"
            world_level = ""
        else:
            world = "Incomplete services"
            world_level = "warn"

        d435i_keys = (
            "infra1_info",
            "infra1_image",
            "infra2_info",
            "infra2_image",
            "imu",
        )
        d435i_count = sum(
            values.get(f"topic_{key}") == "ready" for key in d435i_keys
        )
        scan_ready = values.get("topic_scan") == "ready"
        truth_ready = values.get("topic_truth") == "ready"
        clock_ready = values.get("topic_clock") == "ready"
        outline_ready = values.get("topic_world_outline") == "ready"
        process = values.get("vslam_process", "unknown")
        vo_state = values.get("vo_state", "unknown")

        if process == "running" and vo_state == "1":
            vslam = "Tracking (vo_state=1)"
        elif process == "running":
            vslam = f"Running (vo_state={vo_state})"
        elif process == "stopped":
            vslam = "Stopped"
        else:
            vslam = "Unknown"

        if world_level == "error":
            state, level = "Conflict", "error"
        elif world_level == "warn":
            state, level = "Attention", "warn"
        elif world == "Stopped":
            state, level = "Idle", ""
        elif d435i_count < len(d435i_keys) or not clock_ready:
            state, level = "Sensor issue", "error"
        elif process == "running" and vo_state != "1":
            state, level = "Tracking issue", "warn"
        elif process == "running":
            state, level = "Healthy", "ok"
        else:
            state, level = "Sensors ready", "warn"

        monitor = {
            "enabled": True,
            "state": state,
            "level": level,
            "world": world,
            "gazebo": (
                f"Baseline {baseline_states[0]} / Test {test_states[0]}"
            ),
            "bridge": (
                f"Baseline {baseline_states[1]} / Test {test_states[1]}"
            ),
            "d435i": f"{d435i_count}/{len(d435i_keys)} topics",
            "extra_topics": (
                f"scan {'ready' if scan_ready else 'missing'} / "
                f"native pose {'ready' if truth_ready else 'offline'}"
            ),
            "world_outline": (
                "ready" if outline_ready else "offline (native truth only)"
            ),
            "vslam": vslam,
            "endpoint": f"{host}:{port}",
            "updated": time.strftime("%H:%M:%S"),
        }
        signature = (
            world,
            baseline_states,
            test_states,
            d435i_count,
            scan_ready,
            truth_ready,
            clock_ready,
            outline_ready,
            process,
            vo_state,
        )
        return monitor, signature

    def _monitor_loop(
        self,
        endpoints: list[tuple[str, int]],
        user: str,
        stop_event: threading.Event,
    ) -> None:
        previous_signature = None
        previous_error = None
        sample_count = 0
        while not stop_event.is_set():
            with self.lock:
                busy = self.busy
            if busy:
                stop_event.wait(1)
                continue

            try:
                endpoint = self.find_endpoint_quiet(endpoints)
                if endpoint is None:
                    raise ConnectionError("no configured SSH endpoint responded")
                host, port = endpoint
                monitor, signature = self.collect_monitor_sample(
                    f"{user}@{host}",
                    port,
                    host,
                )
            except (
                ConnectionError,
                RuntimeError,
                subprocess.TimeoutExpired,
                OSError,
            ) as exc:
                error = str(exc)
                monitor = {
                    **self.empty_monitor(),
                    "enabled": True,
                    "state": "Offline",
                    "level": "error",
                    "world": "Unavailable",
                    "endpoint": "No connection",
                    "updated": time.strftime("%H:%M:%S"),
                }
                signature = ("error", error)
                if error != previous_error:
                    self.log(f"Monitor sample failed: {error}", "ERROR")
                previous_error = error
            else:
                previous_error = None
                sample_count += 1
                if signature != previous_signature or sample_count % 4 == 0:
                    level = (
                        "ERROR"
                        if monitor["level"] == "error"
                        else "WARN"
                        if monitor["level"] == "warn"
                        else "OK"
                    )
                    self.log(
                        "Monitor update: "
                        f"world={monitor['world']}; "
                        f"D435i={monitor['d435i']}; "
                        f"LiDAR/native={monitor['extra_topics']}; "
                        f"outline={monitor['world_outline']}; "
                        f"cuVSLAM={monitor['vslam']}.",
                        level,
                    )

            with self.lock:
                if self.monitor_stop_event is not stop_event:
                    return
                self.monitor = monitor
            previous_signature = signature
            stop_event.wait(MONITOR_INTERVAL_SECONDS)

    def request_action(self, payload: dict) -> dict:
        action = str(payload.get("action", "")).strip().lower()
        if action not in {
            "check",
            "reconnect",
            "start",
            "stop",
            "status",
            "vslam_start",
            "vslam_stop",
            "vslam_status",
            "foxglove",
        }:
            raise ValueError("Unknown action")
        endpoints, user = self.settings(payload)
        action_label = self.action_label(action)
        with self.lock:
            if self.busy:
                raise RuntimeError("Another action is already running")
            self.busy = True
            self.status = f"{action_label} requested; locating Jetson..."
            self.level = "busy"
        endpoint_text = ", ".join(f"{host}:{port}" for host, port in endpoints)
        self.log(f"{action_label} requested. Candidates: {endpoint_text}.")
        threading.Thread(
            target=self._run_action,
            args=(action, endpoints, user),
            daemon=True,
        ).start()
        return self.snapshot()

    def request_cleanup(self, payload: dict) -> dict:
        endpoints, user = self.settings(payload)
        with self.lock:
            if self.busy:
                self.pending_cleanup = (endpoints, user)
                self.log(
                    "GUI close cleanup queued until the active action finishes.",
                    "WARN",
                )
                return self.snapshot()
            self.busy = True
            self.status = "GUI closed; cleaning simulation..."
            self.level = "busy"
        self.log("GUI close requested; cleaning the simulation state.")
        threading.Thread(
            target=self._run_action,
            args=("stop", endpoints, user),
            daemon=True,
        ).start()
        return self.snapshot()

    @staticmethod
    def action_label(action: str) -> str:
        return {
            "check": "Check",
            "reconnect": "Reconnect",
            "start": "Simulation start",
            "stop": "Simulation stop",
            "status": "Simulation status",
            "vslam_start": "cuVSLAM start",
            "vslam_stop": "cuVSLAM stop",
            "vslam_status": "cuVSLAM status",
            "foxglove": "Foxglove",
        }[action]

    def _run_action(
        self,
        action: str,
        endpoints: list[tuple[str, int]],
        user: str,
    ) -> None:
        started_at = time.monotonic()
        try:
            endpoint = self.find_endpoint(endpoints)
            if endpoint is None and self.recover_l4t_usb_network(endpoints):
                endpoint = self.find_endpoint(endpoints)
            if endpoint is None:
                raise ConnectionError("Jetson is not reachable. Check the USB/Wi-Fi link.")
            host, port = endpoint
            target = f"{user}@{host}"
            self.log(f"Jetson reachable at {host}:{port}; using {target}.", "OK")

            if action == "check":
                message = f"SSH connection ready: {host}:{port}"
            elif action == "reconnect":
                self.open_terminal(target, port)
                message = f"SSH terminal opened for {target}"
            elif action == "foxglove":
                message = self.open_foxglove(target, port)
            else:
                with self.remote_lock:
                    if action == "start":
                        message = self.start_full_simulation(
                            target, port, host, endpoints, user
                        )
                    elif action == "stop":
                        message = self.stop_full_simulation(target, port)
                    elif action == "status":
                        message = self.full_simulation_status(
                            target, port, host
                        )
                    elif action == "vslam_start":
                        message = self.start_vslam(target, port, host)
                    elif action == "vslam_stop":
                        message = self.stop_vslam(target, port)
                    else:
                        message = self.vslam_status(target, port, host)

            elapsed = time.monotonic() - started_at
            self.log(f"{message}. Completed in {elapsed:.1f}s.", "OK")
            level = "ok"
        except Exception as exc:
            elapsed = time.monotonic() - started_at
            message = f"{self.action_label(action)} failed: {exc}"
            self.log(f"{message}. Failed after {elapsed:.1f}s.", "ERROR")
            level = "error"

        with self.lock:
            self.busy = False
            self.status = message
            self.level = level
            pending_cleanup = self.pending_cleanup
            self.pending_cleanup = None

        if pending_cleanup is not None and action != "stop":
            endpoints, user = pending_cleanup
            with self.lock:
                self.busy = True
                self.status = "GUI closed; cleaning simulation..."
                self.level = "busy"
            self.log("Running queued GUI close cleanup.")
            threading.Thread(
                target=self._run_action,
                args=("stop", endpoints, user),
                daemon=True,
            ).start()

    def cleanup_on_exit(self) -> None:
        """Best-effort cleanup when the local GUI server itself exits."""
        with self.lock:
            if self.cleanup_in_progress:
                return
            self.cleanup_in_progress = True
        try:
            self.log("GUI server exiting; cleaning the simulation state.")
            endpoints = [(L4T_JETSON_ADDRESS, 22), ("192.168.2.2", 22)]
            endpoint = self.find_endpoint(endpoints)
            if endpoint is not None:
                host, port = endpoint
                with self.remote_lock:
                    self.stop_full_simulation(
                        f"tseng@{host}",
                        port,
                    )
            else:
                self.stop_monitor()
                self.stop_foxglove()
        except Exception as exc:
            self.log(f"GUI exit cleanup incomplete: {exc}", "WARN")
        finally:
            with self.lock:
                self.cleanup_in_progress = False

    def start_full_simulation(
        self,
        target: str,
        port: int,
        host: str,
        endpoints: list[tuple[str, int]],
        user: str,
    ) -> str:
        stage = "D435i simulation"
        completed: list[str] = []
        try:
            self.log("[1/4] Starting Gazebo, bridge, and D435i topics.")
            self.backend._start_simulation(target, port, host)
            completed.append("D435i")

            stage = "cuVSLAM"
            self.log("[2/4] Starting cuVSLAM and waiting for vo_state=1.")
            self.start_vslam(target, port, host)
            completed.append("cuVSLAM")

            stage = "Foxglove"
            self.log("[3/4] Opening the Mac Foxglove simulation viewer.")
            self.open_foxglove(target, port)
            completed.append("Foxglove")

            stage = "initial corner mapping route"
            self.log(
                "[4/5] Running the initial corner route: "
                "1 m -> 90 deg -> 0.5 m."
            )
            self.run_mapping_route(target, port)
            completed.append("initial corner route")

            stage = "monitor"
            self.log("[5/5] Starting the SSH low-rate simulation monitor.")
            self.enable_monitor(endpoints, user)
            completed.append("monitor")
        except Exception as exc:
            active = ", ".join(completed) or "none"
            raise RuntimeError(
                f"full start stopped during {stage}; completed stages: {active}; {exc}"
            ) from exc

        return (
            f"Full simulation is running on {host}: "
            "D435i ready, cuVSLAM tracking, Foxglove ready, "
            "initial corner route complete, monitor active"
        )

    def stop_full_simulation(self, target: str, port: int) -> str:
        failures: list[str] = []

        self.log("[1/4] Stopping the SSH low-rate simulation monitor.")
        try:
            self.stop_monitor()
        except Exception as exc:
            failures.append(f"monitor: {exc}")

        self.log("[2/4] Closing the Foxglove simulation tunnel.")
        try:
            self.stop_foxglove()
        except Exception as exc:
            failures.append(f"Foxglove: {exc}")

        self.log("[3/4] Stopping the mapping route.")
        try:
            self.stop_mapping_route(target, port)
        except Exception as exc:
            failures.append(f"mapping route: {exc}")

        self.log("[4/5] Stopping cuVSLAM.")
        try:
            if self.vslam_is_running(target, port):
                self.stop_vslam(target, port)
            else:
                self.log("cuVSLAM is already stopped.")
                self.cleanup_cuvslam_children(target, port)
        except Exception as exc:
            failures.append(f"cuVSLAM: {exc}")

        self.log("[5/5] Stopping Gazebo and the D435i bridge.")
        try:
            self.backend._stop_simulation(target, port)
        except Exception as exc:
            failures.append(f"D435i: {exc}")

        if failures:
            raise RuntimeError("full stop incomplete; " + "; ".join(failures))
        return "Full simulation stopped"

    def full_simulation_status(self, target: str, port: int, host: str) -> str:
        d435i = self.backend._simulation_status(target, port, host)
        vslam = self.vslam_status(target, port, host)
        foxglove = (
            "Foxglove tunnel ready"
            if self.foxglove_tunnel_ready()
            else "Foxglove tunnel stopped"
        )
        return f"{d435i}; {vslam}; {foxglove}"

    @staticmethod
    def foxglove_tunnel_ready() -> bool:
        return subprocess.run(
            [
                "/usr/sbin/lsof",
                "-nP",
                f"-iTCP:{FOXGLOVE_LOCAL_PORT}",
                "-sTCP:LISTEN",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode == 0

    def remote_foxglove_bridge_ready(self, target: str, port: int) -> bool:
        """Check the Jetson-side listener behind the Mac SSH tunnel."""
        result = self.backend.run_ssh(
            target,
            port,
            ["ss", "-ltn"],
            timeout=8,
            check=False,
        )
        if result.returncode != 0:
            return False
        return any(
            f"127.0.0.1:{FOXGLOVE_REMOTE_PORT}" in line
            or f"0.0.0.0:{FOXGLOVE_REMOTE_PORT}" in line
            or f"*:{FOXGLOVE_REMOTE_PORT}" in line
            for line in result.stdout.splitlines()
        )

    def open_foxglove(self, target: str | None = None, port: int | None = None) -> str:
        if not os.path.isfile(FOXGLOVE_LAUNCHER):
            raise RuntimeError(f"Foxglove launcher not found: {FOXGLOVE_LAUNCHER}")

        if self.foxglove_tunnel_ready():
            remote_ready = (
                target is None
                or port is None
                or self.remote_foxglove_bridge_ready(target, port)
            )
            if remote_ready:
                if not os.path.isdir(FOXGLOVE_APP):
                    raise RuntimeError(f"Foxglove app not found: {FOXGLOVE_APP}")
                subprocess.run(["open", "-a", FOXGLOVE_APP], check=True)
                self.log(
                    f"Reusing the healthy Foxglove tunnel on localhost:{FOXGLOVE_LOCAL_PORT}.",
                    "OK",
                )
                return "Foxglove opened using the existing simulation tunnel"

            self.log(
                "Foxglove local tunnel exists but Jetson bridge is unavailable; "
                "rebuilding the tunnel and bridge.",
                "WARN",
            )
            self.stop_foxglove()

        subprocess.run(["open", FOXGLOVE_LAUNCHER], check=True)
        self.log("Foxglove simulation launcher opened in Terminal.", "OK")
        for _ in range(40):
            if self.foxglove_tunnel_ready():
                return "Foxglove launcher opened and simulation tunnel is ready"
            time.sleep(0.5)
        raise RuntimeError("Foxglove tunnel did not become ready within 20 seconds")

    def stop_foxglove(self) -> None:
        listeners = subprocess.run(
            [
                "/usr/sbin/lsof",
                "-nP",
                "-t",
                f"-iTCP:{FOXGLOVE_LOCAL_PORT}",
                "-sTCP:LISTEN",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        pids = [int(pid) for pid in listeners.stdout.split() if pid.isdigit()]
        if not pids:
            self.log("Foxglove tunnel is already stopped.")
            return

        expected_forward = (
            f"127.0.0.1:{FOXGLOVE_LOCAL_PORT}:127.0.0.1:8765"
        )
        for pid in pids:
            command = subprocess.run(
                ["ps", "-p", str(pid), "-o", "command="],
                capture_output=True,
                text=True,
                check=False,
            ).stdout.strip()
            if "ssh " not in command or expected_forward not in command:
                raise RuntimeError(
                    f"port {FOXGLOVE_LOCAL_PORT} belongs to an unexpected process"
                )
            os.kill(pid, signal.SIGTERM)

        for _ in range(20):
            if not self.foxglove_tunnel_ready():
                self.log("Foxglove tunnel stopped.", "OK")
                return
            time.sleep(0.25)
        raise RuntimeError("Foxglove tunnel is still listening")

    def vslam_process_running(self, target: str, port: int) -> bool:
        result = self.backend.run_ssh(
            target,
            port,
            [
                "docker",
                "exec",
                ISAAC_CONTAINER,
                "pgrep",
                "-f",
                VSLAM_LAUNCH,
            ],
            check=False,
        )
        return result.returncode == 0

    def ensure_isaac_container_running(self, target: str, port: int) -> None:
        """Ensure the persistent Isaac ROS dev container is running.

        The container is deliberately not a boot-time service.  A Jetson
        reboot therefore leaves it in the ``exited`` state even though the
        Gazebo/D435i transient units can be started normally.  Starting
        cuVSLAM with ``docker exec`` in that state produces a misleading
        partial-start failure and leaves Foxglove connected without map
        topics.  Recover an existing container here before any exec call.
        """
        inspect = self.backend.run_ssh(
            target,
            port,
            [
                "docker",
                "inspect",
                "--format={{.State.Running}}",
                ISAAC_CONTAINER,
            ],
            timeout=10,
            check=False,
        )
        state = inspect.stdout.strip().lower()
        if inspect.returncode != 0:
            detail = inspect.stderr.strip() or inspect.stdout.strip()
            raise RuntimeError(
                f"Isaac ROS container {ISAAC_CONTAINER!r} is missing or "
                f"cannot be inspected: {detail or 'unknown docker error'}"
            )
        if state == "true":
            return
        if state != "false":
            raise RuntimeError(
                f"Isaac ROS container {ISAAC_CONTAINER!r} has unexpected "
                f"docker state {state!r}"
            )

        self.log(
            f"Isaac ROS container {ISAAC_CONTAINER} is stopped; starting it "
            "before cuVSLAM."
        )
        self.backend.run_ssh(
            target,
            port,
            ["docker", "start", ISAAC_CONTAINER],
            timeout=30,
        )
        for _ in range(30):
            ready = self.backend.run_ssh(
                target,
                port,
                [
                    "docker",
                    "exec",
                    ISAAC_CONTAINER,
                    "test",
                    "-f",
                    "/workspaces/isaac_ros-dev/install/setup.bash",
                ],
                timeout=8,
                check=False,
            )
            if ready.returncode == 0:
                return
            time.sleep(1.0)
        raise RuntimeError(
            f"Isaac ROS container {ISAAC_CONTAINER!r} started but its "
            "workspace setup.bash did not become ready"
        )

    def prepare_active_world(self, target: str, port: int) -> None:
        result = self.backend.run_ssh(
            target,
            port,
            [
                "install",
                "-D",
                "-m",
                "0644",
                VSLAM_WORLD_HOST,
                "/home/tseng/isaac_ros_ws/data/gui_active_world.sdf",
            ],
            timeout=10,
            check=False,
        )
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(
                "Active dense-turn SDF could not be staged for Foxglove "
                f"ground truth: {detail or 'unknown remote error'}"
            )

    def vslam_is_running(self, target: str, port: int) -> bool:
        return self.vslam_process_running(target, port)

    @staticmethod
    def vslam_shell() -> str:
        return "\n".join(
            [
                "set -e",
                "source /opt/ros/humble/setup.bash",
                "source /workspaces/isaac_ros-dev/install/setup.bash",
                "export ROS_DOMAIN_ID=43",
                "export ROS2CLI_DISABLE_DAEMON=1",
                (
                    "exec ros2 launch isaac_ros_visual_slam "
                    f"isaac_ros_visual_slam_d435i_sim.launch.py "
                    f"camera_x:={VSLAM_CAMERA_X} "
                    f"enable_mapping:={VSLAM_ENABLE_MAPPING} "
                    f"enable_imu_fusion:={VSLAM_ENABLE_IMU_FUSION} "
                    "enable_debug_mode:=false "
                    f"enable_slam_visualization:={VSLAM_ENABLE_SLAM_VISUALIZATION} "
                    f"enable_observations_view:={VSLAM_ENABLE_OBSERVATIONS_VIEW} "
                    f"world_sdf:={VSLAM_WORLD_CONTAINER} "
                    f"> {VSLAM_LOG} 2>&1"
                ),
            ]
        )

    def vslam_status_sample(self, target: str, port: int) -> str:
        shell = "\n".join(
            [
                "source /opt/ros/humble/setup.bash",
                "source /workspaces/isaac_ros-dev/install/setup.bash",
                "export ROS_DOMAIN_ID=43",
                "export ROS2CLI_DISABLE_DAEMON=1",
                "timeout 8 ros2 topic echo --once /visual_slam/status",
            ]
        )
        try:
            result = self.backend.run_ssh(
                target,
                port,
                ["docker", "exec", ISAAC_CONTAINER, "bash", "-lc", shell],
                timeout=16,
                check=False,
            )
        except subprocess.TimeoutExpired:
            self.log("cuVSLAM status sample timed out; retrying.", "WARN")
            return ""
        return result.stdout.strip() if result.returncode == 0 else ""

    def vslam_logs(self, target: str, port: int) -> str:
        result = self.backend.run_ssh(
            target,
            port,
            [
                "docker",
                "exec",
                ISAAC_CONTAINER,
                "tail",
                "-n",
                "30",
                VSLAM_LOG,
            ],
            check=False,
        )
        return result.stdout.strip() or result.stderr.strip()

    def start_vslam(self, target: str, port: int, host: str) -> str:
        # start_full_simulation() has just completed the D435i topic and live
        # image readiness gate.  Repeating that SSH/ROS CLI probe here can
        # race Foxy discovery and falsely block cuVSLAM before it starts.
        self.ensure_isaac_container_running(target, port)
        self.prepare_active_world(target, port)

        if self.vslam_is_running(target, port):
            sample = self.vslam_status_sample(target, port)
            if "vo_state: 1" in sample:
                return f"cuVSLAM stereo tracking is already running on {host}"
            self.log("Removing an incomplete cuVSLAM launch before restart.", "WARN")
            self.stop_vslam(target, port)

        self.log(
            "Starting cuVSLAM 11.4 in stereo mode; IMU fusion remains disabled "
            "until the simulated optical-to-IMU extrinsic is validated."
        )
        run_result = self.backend.run_ssh(
            target,
            port,
            [
                "docker",
                "exec",
                "-d",
                ISAAC_CONTAINER,
                "bash",
                "-lc",
                self.vslam_shell(),
            ],
            timeout=12,
        )
        self.log(
            "docker exec: "
            + (run_result.stdout.strip() or run_result.stderr.strip() or "launch submitted")
        )

        for _ in range(12):
            process_running = self.vslam_process_running(target, port)
            if not process_running:
                self.log("cuVSLAM launch process exited before tracker readiness.", "ERROR")
                break
            sample = self.vslam_status_sample(target, port)
            if "vo_state: 1" in sample:
                self.log("cuVSLAM reported vo_state=1 (visual tracking success).", "OK")
                return f"cuVSLAM stereo tracking is running on {host}"
            self.log("cuVSLAM process active; waiting for tracker output.")
            time.sleep(1)

        logs = self.vslam_logs(target, port)
        self.log(f"Recent cuVSLAM runtime log:\n{logs}", "ERROR")
        self.stop_vslam(target, port)
        raise RuntimeError("cuVSLAM did not report vo_state=1")

    def stop_vslam(self, target: str, port: int) -> str:
        self.log("Stopping cuVSLAM launch inside the Isaac ROS container.")
        self.backend.run_ssh(
            target,
            port,
            [
                "docker",
                "exec",
                ISAAC_CONTAINER,
                "pkill",
                "-INT",
                "-f",
                VSLAM_LAUNCH,
            ],
            timeout=8,
            check=False,
        )
        time.sleep(0.5)
        if self.vslam_process_running(target, port):
            self.backend.run_ssh(
                target,
                port,
                [
                    "docker",
                    "exec",
                    ISAAC_CONTAINER,
                    "pkill",
                    "-TERM",
                    "-f",
                    VSLAM_LAUNCH,
                ],
                timeout=8,
                check=False,
            )
            time.sleep(0.5)
        if self.vslam_process_running(target, port):
            raise RuntimeError("cuVSLAM process is still running")
        self.cleanup_cuvslam_children(target, port)
        self.log("cuVSLAM process stopped.", "OK")
        return "cuVSLAM stopped"

    def cleanup_cuvslam_children(self, target: str, port: int) -> None:
        """Stop launch children that can survive a ROS launch shutdown.

        A composable container or adapter left behind keeps publishing TF,
        holding the bag, and sometimes retaining a large debug file.  The
        patterns are deliberately limited to this simulation's node names.
        """
        shell = r"""
set +e
patterns=(
  '[r]ight_camera_info_adapter'
  '[d]435i_sim_visual_slam_container'
  '[w]orld_outline_publisher'
  '[s]imulation_native_pose_relay.py'
  '[b]ase_to_camera'
  '[c]amera_to_infra1'
  '[c]amera_to_infra2'
  '[i]nfra1_to_infra1_optical'
  '[i]nfra2_to_infra2_optical'
  '[c]amera_to_imu'
  '[i]mu_to_simulated_imu'
  '[b]ase_to_lidar'
  '[l]idar_to_simulated_lidar'
)
for pattern in "${patterns[@]}"; do
  pids="$(pgrep -f "$pattern" || true)"
  [ -z "$pids" ] || kill -TERM $pids 2>/dev/null || true
done
sleep 0.7
for pattern in "${patterns[@]}"; do
  pids="$(pgrep -f "$pattern" || true)"
  [ -z "$pids" ] || kill -KILL $pids 2>/dev/null || true
done
"""
        result = self.backend.run_ssh(
            target,
            port,
            ["docker", "exec", ISAAC_CONTAINER, "bash", "-lc", shell],
            timeout=15,
            check=False,
        )
        if result.returncode:
            detail = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(
                "cuVSLAM child cleanup failed: "
                f"{detail or f'exit code {result.returncode}'}"
            )
        self.log("cuVSLAM child nodes and simulation TF publishers cleaned.", "OK")

    def mapping_route_running(self, target: str, port: int) -> bool:
        result = self.backend.run_ssh(
            target,
            port,
            [
                "docker",
                "exec",
                ISAAC_CONTAINER,
                "pgrep",
                "-f",
                "[r]un_container_straight_mapping_trial.sh",
            ],
            timeout=8,
            check=False,
        )
        return result.returncode == 0

    def stop_mapping_route(self, target: str, port: int) -> None:
        if not self.mapping_route_running(target, port):
            return
        self.log("Stopping the active initial corner mapping route.")
        self.backend.run_ssh(
            target,
            port,
            [
                "docker",
                "exec",
                ISAAC_CONTAINER,
                "pkill",
                "-INT",
                "-f",
                "[r]un_container_straight_mapping_trial.sh",
            ],
            timeout=8,
            check=False,
        )
        for _ in range(10):
            if not self.mapping_route_running(target, port):
                return
            time.sleep(0.5)

    def run_mapping_route(self, target: str, port: int) -> str:
        run_id = time.strftime("gui_native_corner_%Y%m%d_%H%M%S")
        output_dir = f"/workspaces/isaac_ros-dev/data/{run_id}"
        route_log = f"{output_dir}/route.log"
        route_shell = "\n".join(
            [
                "set -e",
                "source /opt/ros/humble/setup.bash",
                "source /workspaces/isaac_ros-dev/install/setup.bash",
                "export ROS_DOMAIN_ID=43",
                "export ROS2CLI_DISABLE_DAEMON=1",
                "export CUVSLAM_MOTION_PROFILE=native_corner",
                # Dense v3's validated pose baseline is 20 deg/s.  The old
                # 5 deg/s route made the turn under-observable and produced
                # large false translation/roll-pitch drift.
                "export CUVSLAM_TURN_RATE_DEG_S=20.0",
                "export CUVSLAM_LINEAR_SPEED=0.12",
                # The dense-turn world brings the lidar to 0.292 m at the
                # planned second leg.  Keep a real clearance gate, but set
                # it below that measured safe path so the requested 0.5 m
                # leg is not rejected by an overly conservative threshold.
                "export CUVSLAM_STOP_DISTANCE=0.25",
                f"mkdir -p {output_dir}",
                f"{MAPPING_ROUTE_SCRIPT} {output_dir} > {route_log} 2>&1",
            ]
        )
        self.log(
            "Starting the initial corner mapping route: "
            "1 m -> 90 deg -> 0.5 m."
        )
        self.backend.run_ssh(
            target,
            port,
            [
                "docker",
                "exec",
                "-d",
                ISAAC_CONTAINER,
                "bash",
                "-lc",
                route_shell,
            ],
            timeout=12,
        )

        for _ in range(180):
            if not self.mapping_route_running(target, port):
                break
            if _ % 15 == 0:
                self.log(f"Mapping route active; output={output_dir}.")
            time.sleep(1.0)

        log_result = self.backend.run_ssh(
            target,
            port,
            ["docker", "exec", ISAAC_CONTAINER, "tail", "-n", "80", route_log],
            timeout=10,
            check=False,
        )
        route_output = log_result.stdout.strip() or log_result.stderr.strip()
        if "CONTAINER_STRAIGHT_MAPPING_COMPLETE" not in route_output:
            self.log(f"Mapping route did not complete cleanly:\n{route_output}", "ERROR")
            raise RuntimeError(
                f"initial corner mapping route failed; output={output_dir}"
            )
        self.log(
            f"Initial corner mapping route completed successfully. Rosbag and route log: {output_dir}",
            "OK",
        )
        return output_dir

    def vslam_status(self, target: str, port: int, host: str) -> str:
        process_running = self.vslam_process_running(target, port)
        self.log(f"cuVSLAM status: process_running={process_running}.")
        if not process_running:
            return "cuVSLAM is stopped"
        sample = self.vslam_status_sample(target, port)
        if "vo_state: 1" in sample:
            self.log("cuVSLAM tracker reports vo_state=1.", "OK")
            return f"cuVSLAM stereo tracking is healthy on {host}"
        if "vo_state:" in sample:
            state_line = next(
                line.strip() for line in sample.splitlines() if "vo_state:" in line
            )
            self.log(f"cuVSLAM tracker reports {state_line}.", "WARN")
            return f"cuVSLAM is running with {state_line}"
        return "cuVSLAM process is running; tracker output is not ready"

    def open_terminal(self, target: str, port: int) -> None:
        command = [
            "ssh",
            "-tt",
            "-p",
            str(port),
            "-o",
            "ConnectTimeout=8",
            "-o",
            "ServerAliveInterval=15",
            "-o",
            "ServerAliveCountMax=3",
            "-o",
            "TCPKeepAlive=yes",
            "-o",
            "ControlMaster=no",
            "-o",
            "ControlPath=none",
            target,
        ]
        terminal_script = " ".join(json.dumps(part) for part in command)
        osa = [
            'tell application "Terminal"',
            f"do script {json.dumps(terminal_script)}",
            "activate",
            "end tell",
        ]
        result = subprocess.run(
            ["osascript", "-e", "\n".join(osa)],
            capture_output=True,
            text=True,
        )
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or "Could not open Terminal")


CONTROLLER = Controller()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args) -> None:
        return

    def send_bytes(self, status: int, content_type: str, payload: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def send_json(self, status: int, payload: dict) -> None:
        self.send_bytes(
            status,
            "application/json; charset=utf-8",
            json.dumps(payload).encode("utf-8"),
        )

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/":
            self.send_bytes(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        elif path == "/api/snapshot":
            self.send_json(200, CONTROLLER.snapshot())
        else:
            self.send_json(404, {"error": "Not found"})

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            if path == "/api/action":
                length = min(int(self.headers.get("Content-Length", "0")), 8192)
                payload = json.loads(self.rfile.read(length) or b"{}")
                self.send_json(202, CONTROLLER.request_action(payload))
            elif path == "/api/cleanup":
                length = min(int(self.headers.get("Content-Length", "0")), 8192)
                payload = json.loads(self.rfile.read(length) or b"{}")
                self.send_json(202, CONTROLLER.request_cleanup(payload))
            elif path == "/api/monitor/start":
                length = min(int(self.headers.get("Content-Length", "0")), 8192)
                payload = json.loads(self.rfile.read(length) or b"{}")
                self.send_json(202, CONTROLLER.start_monitor(payload))
            elif path == "/api/monitor/stop":
                self.send_json(200, CONTROLLER.stop_monitor())
            elif path == "/api/logs/clear":
                self.send_json(200, CONTROLLER.clear_logs())
            else:
                self.send_json(404, {"error": "Not found"})
        except (ValueError, RuntimeError, json.JSONDecodeError) as exc:
            CONTROLLER.log(str(exc), "ERROR")
            self.send_json(400, {"error": str(exc)})


def open_app_window(url: str) -> None:
    chrome = "/Applications/Google Chrome.app"
    if os.path.isdir(chrome):
        subprocess.Popen(
            [
                "open",
                "-na",
                "Google Chrome",
                "--args",
                f"--app={url}",
                "--user-data-dir=/private/tmp/cuvslam-jetson-gui-chrome",
                "--window-size=1180,720",
            ]
        )
    else:
        webbrowser.open(url)


def main() -> None:
    url = f"http://{HOST}:{PORT}/"
    try:
        server = ThreadingHTTPServer((HOST, PORT), Handler)
    except OSError:
        open_app_window(url)
        return
    threading.Timer(0.4, open_app_window, args=(url,)).start()

    def handle_shutdown(signum, frame) -> None:
        CONTROLLER.cleanup_on_exit()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, handle_shutdown)
    signal.signal(signal.SIGINT, handle_shutdown)
    atexit.register(CONTROLLER.cleanup_on_exit)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
