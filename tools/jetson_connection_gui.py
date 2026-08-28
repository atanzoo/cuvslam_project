#!/usr/bin/env python3
"""Small macOS GUI for reconnecting to the Jetson without legacy scripts."""

from __future__ import annotations

import json
import platform
import queue
import shlex
import shutil
import socket
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox


GAZEBO_UNIT = "cuvslam-d435i-gazebo.service"
BRIDGE_UNIT = "cuvslam-d435i-bridge.service"
SIM_UNITS = (GAZEBO_UNIT, BRIDGE_UNIT)
SIM_PARTITION = "cuvslam_d435i_gui"
SIM_DOMAIN_ID = "43"
SIM_WORLD = (
    "/home/tseng/jetson_slam_ws/install/slam_gazebo/share/slam_gazebo/worlds/"
    "cuvslam_mapping_observable_640x360_range2p5_dense_turn.sdf"
)
SIM_NATIVE_POSE_TOPIC = (
    "/world/cuvslam_mapping_simple/dynamic_pose/info"
)
EXPECTED_SIM_TOPICS = {
    "/d435i/infra1/camera_info",
    "/d435i/infra1/image_rect_raw",
    "/d435i/infra2/camera_info",
    "/d435i/infra2/image_rect_raw",
    "/d435i/imu",
}


class JetsonConnectionApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("cuVSLAM Jetson Connection")
        self.resizable(True, True)
        self.minsize(900, 430)

        self.host = tk.StringVar(value="192.168.55.1,192.168.2.2")
        self.user = tk.StringVar(value="tseng")
        self.port = tk.StringVar(value="22")
        self.status = tk.StringVar(value="Ready")
        self.sim_status = tk.StringVar(value="Simulation status not checked")
        self.sim_buttons: list[tk.Button] = []
        self.log_text = None
        self.ui_events = queue.Queue()

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.close_window)
        self.log_message(
            f"GUI ready. ROS domain {SIM_DOMAIN_ID}, partition {SIM_PARTITION}."
        )
        self.after(100, self._drain_ui_events)
        self.after_idle(self._bring_to_front)

    def _bring_to_front(self) -> None:
        """Bring the Tk window above the Terminal that launched the .command file."""
        self.deiconify()
        self.lift()
        self.focus_force()
        if platform.system() == "Darwin":
            self.attributes("-topmost", True)
            self.after(400, lambda: self.attributes("-topmost", False))

    def _build_ui(self) -> None:
        colors = {
            "window": "#f4f5f7",
            "text": "#172033",
            "muted": "#4b5563",
            "field": "#ffffff",
            "button": "#e2e8f0",
            "primary": "#1677ff",
            "primary_active": "#0f5fc7",
            "border": "#c5ccd6",
        }
        self.configure(background=colors["window"])

        root = tk.Frame(self, background=colors["window"], padx=18, pady=18)
        root.grid(row=0, column=0, sticky="nsew")

        tk.Label(
            root,
            text="Jetson connection",
            font=("Helvetica", 17, "bold"),
            background=colors["window"],
            foreground=colors["text"],
        ).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 14)
        )

        fields = (("Host(s)", self.host), ("User", self.user), ("SSH port", self.port))
        for row, (label, variable) in enumerate(fields, start=1):
            tk.Label(
                root,
                text=label,
                background=colors["window"],
                foreground=colors["text"],
            ).grid(row=row, column=0, sticky="w", padx=(0, 12), pady=5)
            tk.Entry(
                root,
                textvariable=variable,
                width=30,
                background=colors["field"],
                foreground=colors["text"],
                insertbackground=colors["text"],
                relief="solid",
                borderwidth=1,
                highlightthickness=1,
                highlightbackground=colors["border"],
                highlightcolor=colors["primary"],
            ).grid(row=row, column=1, sticky="ew", pady=5, ipady=4)

        button_style = {
            "background": colors["button"],
            "foreground": colors["text"],
            "activebackground": "#d4dbe5",
            "activeforeground": colors["text"],
            "relief": "raised",
            "borderwidth": 1,
            "highlightbackground": colors["window"],
            "padx": 10,
            "pady": 5,
        }
        primary_button_style = {
            **button_style,
            "background": colors["primary"],
            "foreground": "#ffffff",
            "activebackground": colors["primary_active"],
            "activeforeground": "#ffffff",
        }

        buttons = tk.Frame(root, background=colors["window"])
        buttons.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(14, 8))
        tk.Button(
            buttons,
            text="Check connection",
            command=self.check_connection,
            **button_style,
        ).grid(row=0, column=0, padx=(0, 8))
        tk.Button(
            buttons,
            text="Reconnect SSH",
            command=self.open_ssh_terminal,
            **button_style,
        ).grid(row=0, column=1)

        tk.Frame(root, background=colors["border"], height=1).grid(
            row=5, column=0, columnspan=2, sticky="ew", pady=8
        )
        tk.Label(
            root,
            text="D435i simulation",
            font=("Helvetica", 13, "bold"),
            background=colors["window"],
            foreground=colors["text"],
        ).grid(
            row=6, column=0, columnspan=2, sticky="w", pady=(4, 8)
        )

        sim_buttons = tk.Frame(root, background=colors["window"])
        sim_buttons.grid(row=7, column=0, columnspan=2, sticky="ew")
        start_button = tk.Button(
            sim_buttons,
            text="Start simulation",
            command=self.start_simulation,
            **primary_button_style,
        )
        stop_button = tk.Button(
            sim_buttons,
            text="Stop",
            command=self.stop_simulation,
            **button_style,
        )
        status_button = tk.Button(
            sim_buttons,
            text="Refresh status",
            command=self.refresh_simulation_status,
            **button_style,
        )
        start_button.grid(row=0, column=0, padx=(0, 8))
        stop_button.grid(row=0, column=1, padx=(0, 8))
        status_button.grid(row=0, column=2)
        self.sim_buttons = [start_button, stop_button, status_button]

        tk.Label(
            root,
            textvariable=self.sim_status,
            wraplength=420,
            justify="left",
            background=colors["window"],
            foreground=colors["muted"],
        ).grid(
            row=8, column=0, columnspan=2, sticky="w", pady=(8, 4)
        )
        tk.Frame(root, background=colors["border"], height=1).grid(
            row=9, column=0, columnspan=2, sticky="ew", pady=8
        )
        tk.Label(
            root,
            textvariable=self.status,
            wraplength=310,
            justify="left",
            background=colors["window"],
            foreground=colors["muted"],
        ).grid(row=10, column=0, columnspan=2, sticky="w")

        tk.Frame(root, background=colors["border"], width=1).grid(
            row=0,
            column=2,
            rowspan=11,
            sticky="ns",
            padx=(18, 18),
        )

        log_panel = tk.Frame(root, background=colors["window"])
        log_panel.grid(row=0, column=3, rowspan=11, sticky="nsew")
        tk.Label(
            log_panel,
            text="Work log",
            font=("Helvetica", 15, "bold"),
            background=colors["window"],
            foreground=colors["text"],
        ).grid(row=0, column=0, sticky="w", pady=(0, 10))

        log_actions = tk.Frame(log_panel, background=colors["window"])
        log_actions.grid(row=0, column=1, columnspan=2, sticky="e", pady=(0, 8))
        tk.Button(
            log_actions,
            text="Copy log",
            command=self.copy_log,
            **button_style,
        ).grid(row=0, column=0, padx=(0, 8))
        tk.Button(
            log_actions,
            text="Clear log",
            command=self.clear_log,
            **button_style,
        ).grid(row=0, column=1)

        log_text = tk.Text(
            log_panel,
            width=62,
            height=22,
            wrap="word",
            background="#111827",
            foreground="#e5e7eb",
            insertbackground="#e5e7eb",
            selectbackground="#315b88",
            selectforeground="#ffffff",
            font=("Menlo", 10),
            relief="solid",
            borderwidth=1,
            padx=10,
            pady=10,
            state="disabled",
        )
        log_scrollbar = tk.Scrollbar(log_panel, command=log_text.yview)
        log_text.configure(yscrollcommand=log_scrollbar.set)
        log_text.grid(row=1, column=0, columnspan=2, sticky="nsew")
        log_scrollbar.grid(row=1, column=2, sticky="ns")
        self.log_text = log_text

        root.columnconfigure(1, weight=0)
        root.columnconfigure(3, weight=1)
        root.rowconfigure(10, weight=1)
        log_panel.columnconfigure(0, weight=1)
        log_panel.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

    def log_message(self, message: str, level: str = "INFO") -> None:
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] [{level}] {message.rstrip()}\n"
        if threading.current_thread() is threading.main_thread():
            self._append_log(line)
        else:
            self.ui_events.put(("log", line))

    def _drain_ui_events(self) -> None:
        try:
            while True:
                event = self.ui_events.get_nowait()
                if event[0] == "log":
                    self._append_log(event[1])
                elif event[0] == "finish":
                    self._finish_simulation_action(event[1], event[2])
        except queue.Empty:
            pass
        self.after(100, self._drain_ui_events)

    def _append_log(self, line: str) -> None:
        if self.log_text is None:
            return
        self.log_text.configure(state="normal")
        self.log_text.insert("end", line)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self) -> None:
        if self.log_text is None:
            return
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self.status.set("Work log cleared")

    def copy_log(self) -> None:
        if self.log_text is None:
            return
        content = self.log_text.get("1.0", "end-1c")
        self.clipboard_clear()
        self.clipboard_append(content)
        self.update_idletasks()
        self.status.set("Work log copied to clipboard")

    def endpoints(self) -> list[tuple[str, int]]:
        hosts = [item.strip() for item in self.host.get().split(",") if item.strip()]
        if not hosts:
            raise ValueError("At least one host is required")
        try:
            port = int(self.port.get().strip())
        except ValueError as exc:
            raise ValueError("SSH port must be a number") from exc
        if not 1 <= port <= 65535:
            raise ValueError("SSH port must be between 1 and 65535")
        return [(host, port) for host in hosts]

    def find_reachable_endpoint(
        self, endpoints: list[tuple[str, int]] | None = None
    ) -> tuple[str, int] | None:
        candidates = self.endpoints() if endpoints is None else endpoints
        for host, port in candidates:
            try:
                with socket.create_connection((host, port), timeout=2):
                    return host, port
            except OSError:
                continue
        return None

    def connection_target(self) -> tuple[str, int, str]:
        endpoint = self.find_reachable_endpoint()
        if endpoint is None:
            raise ConnectionError("Jetson is not reachable. Check the USB/Wi-Fi link.")
        user = self.user.get().strip()
        if not user:
            raise ValueError("User is required")
        host, port = endpoint
        return host, port, f"{user}@{host}"

    @staticmethod
    def ssh_command(target: str, port: int, remote_args: list[str]) -> list[str]:
        return [
            "ssh",
            "-p",
            str(port),
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=8",
            "-o",
            "ServerAliveInterval=15",
            "-o",
            "ServerAliveCountMax=3",
            "-o",
            "ControlMaster=no",
            "-o",
            "ControlPath=none",
            "-o",
            "StrictHostKeyChecking=accept-new",
            target,
            shlex.join(remote_args),
        ]

    def run_ssh(
        self,
        target: str,
        port: int,
        remote_args: list[str],
        *,
        timeout: int = 30,
        check: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        result = subprocess.run(
            self.ssh_command(target, port, remote_args),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        if check and result.returncode:
            detail = result.stderr.strip() or result.stdout.strip() or f"exit code {result.returncode}"
            raise RuntimeError(detail)
        return result

    def service_states(self, target: str, port: int) -> dict[str, str]:
        result = self.run_ssh(
            target,
            port,
            ["systemctl", "--user", "is-active", *SIM_UNITS],
            check=False,
        )
        states = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        states.extend(["unknown"] * (len(SIM_UNITS) - len(states)))
        return dict(zip(SIM_UNITS, states))

    def service_logs(self, target: str, port: int) -> str:
        result = self.run_ssh(
            target,
            port,
            [
                "journalctl",
                "--user",
                f"--unit={GAZEBO_UNIT}",
                f"--unit={BRIDGE_UNIT}",
                "--lines=12",
                "--no-pager",
            ],
            check=False,
        )
        journal = result.stdout.strip() or result.stderr.strip()
        files = self.run_ssh(
            target,
            port,
            [
                "bash",
                "-lc",
                "for f in /home/tseng/isaac_ros_ws/data/gui_service_logs/gazebo.log "
                "/home/tseng/isaac_ros_ws/data/gui_service_logs/bridge.log; do "
                "if test -f \"$f\"; then echo \"--- $f ---\"; tail -n 40 \"$f\"; fi; done",
            ],
            check=False,
        )
        persistent = files.stdout.strip() or files.stderr.strip()
        return "\n".join(part for part in (journal, persistent) if part)

    def sensor_topics_ready(self, target: str, port: int) -> bool:
        shell = "\n".join(
            [
                "source /opt/ros/foxy/setup.bash",
                f"export ROS_DOMAIN_ID={SIM_DOMAIN_ID}",
                "export ROS2CLI_DISABLE_DAEMON=1",
                "ros2 topic list",
            ]
        )
        result = self.run_ssh(
            target,
            port,
            ["/bin/bash", "-lc", shell],
            timeout=10,
            check=False,
        )
        topics = {line.strip() for line in result.stdout.splitlines()}
        if result.returncode != 0 or not EXPECTED_SIM_TOPICS.issubset(topics):
            return False

        image_check = self.run_ssh(
            target,
            port,
            [
                "/bin/bash",
                "-lc",
                (
                    "source /opt/ros/foxy/setup.bash; "
                    f"export ROS_DOMAIN_ID={SIM_DOMAIN_ID}; "
                    "export ROS2CLI_DISABLE_DAEMON=1; "
                    "timeout 6 ros2 topic echo "
                    "/d435i/infra1/image_rect_raw "
                    "--qos-reliability reliable --no-arr"
                ),
            ],
            timeout=10,
            check=False,
        )
        # `timeout` exits with 124 after the first live sample; that is a
        # successful readiness check as long as ros2 printed an image header.
        return "header:" in image_check.stdout

    def stop_units(self, target: str, port: int) -> None:
        self.run_ssh(
            target,
            port,
            ["systemctl", "--user", "stop", *reversed(SIM_UNITS)],
            timeout=20,
            check=False,
        )

    def cleanup_regenerable_simulation_state(
        self,
        target: str,
        port: int,
    ) -> None:
        """Remove only regenerable logs and stale transport dump writers.

        Experiment bags, SDFs, reports, and mapping output are intentionally
        outside this cleanup.  The transport recorder processes are stopped
        before their files are removed so an unlinked file cannot keep the
        Jetson disk full through an open file descriptor.
        """
        shell = r"""
set +e
transport_pattern='[i]gn-transport-topic.*--json-output'
transport_pids="$(pgrep -f "$transport_pattern" || true)"
if [ -n "$transport_pids" ]; then
  kill -TERM $transport_pids 2>/dev/null || true
  sleep 0.5
  transport_pids="$(pgrep -f "$transport_pattern" || true)"
  [ -z "$transport_pids" ] || kill -KILL $transport_pids 2>/dev/null || true
fi

# ROS2 launch logs are reproducible.  Keep recent logs for diagnosis and
# remove only directories older than seven days.
if [ -d "$HOME/.ros/log" ]; then
  find "$HOME/.ros/log" -mindepth 1 -maxdepth 1 -type d -mtime +7 \
    -exec rm -rf -- {} + 2>/dev/null || true
fi
rm -f /tmp/cuvslam-d435i-vslam.log /tmp/cuvslam-foxglove-bridge.log

df -Pk / | tail -n 1
"""
        result = self.run_ssh(
            target,
            port,
            ["/bin/bash", "-lc", shell],
            timeout=15,
            check=False,
        )
        if result.returncode:
            detail = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(
                "simulation cleanup failed: "
                f"{detail or f'exit code {result.returncode}'}"
            )

    def disk_preflight(self, target: str, port: int) -> None:
        """Fail early only when the root filesystem is genuinely too full."""
        result = self.run_ssh(
            target,
            port,
            ["df", "-Pk", "/"],
            timeout=8,
            check=False,
        )
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or "could not inspect Jetson disk")
        rows = [line.split() for line in result.stdout.splitlines() if line.strip()]
        if len(rows) < 2 or len(rows[-1]) < 4:
            raise RuntimeError("Jetson disk check returned an unexpected result")
        try:
            available_kb = int(rows[-1][3])
        except ValueError as exc:
            raise RuntimeError("Jetson disk check returned a non-numeric value") from exc
        available_gib = available_kb / 1024 / 1024
        if available_kb < 768 * 1024:
            raise RuntimeError(
                "Jetson root filesystem has only "
                f"{available_gib:.2f} GiB free; clean storage before starting Gazebo"
            )
        if available_kb < 2 * 1024 * 1024:
            self.log_message(
                f"Jetson disk preflight: {available_gib:.2f} GiB free; "
                "simulation can run but old bags should be archived.",
                "WARN",
            )
        else:
            self.log_message(f"Jetson disk preflight: {available_gib:.2f} GiB free.")

    @staticmethod
    def simulation_shell(launch_file: str, log_file: str) -> str:
        return "\n".join(
            [
                "set -e",
                f"mkdir -p {shlex.quote(log_file.rsplit('/', 1)[0])}",
                "source /opt/ros/foxy/setup.bash",
                "source /home/tseng/jetson_slam_ws/install/setup.bash",
                f"export ROS_DOMAIN_ID={SIM_DOMAIN_ID}",
                (
                    f"exec ros2 launch slam_gazebo {launch_file} "
                    f"ign_partition:={SIM_PARTITION}"
                    + (
                        f" world:={SIM_WORLD}"
                        if launch_file == "gz_sim.launch.py"
                        else ""
                    )
                    + (
                        " enable_native_pose:=true"
                        f" native_pose_topic:={SIM_NATIVE_POSE_TOPIC}"
                        if launch_file == "gz_bridge_portable.launch.py"
                        else ""
                    )
                    + (" use_gui:=false" if launch_file == "gz_sim.launch.py" else "")
                    + f" >{shlex.quote(log_file)} 2>&1"
                ),
            ]
        )

    def start_unit(self, target: str, port: int, unit: str, launch_file: str) -> None:
        log_file = (
            "/home/tseng/isaac_ros_ws/data/gui_service_logs/gazebo.log"
            if launch_file == "gz_sim.launch.py"
            else "/home/tseng/isaac_ros_ws/data/gui_service_logs/bridge.log"
        )
        self.run_ssh(
            target,
            port,
            [
                "systemd-run",
                "--user",
                f"--unit={unit.removesuffix('.service')}",
                "--collect",
                "--property=KillMode=control-group",
                "--property=TimeoutStopSec=8s",
                "/bin/bash",
                "-lc",
                self.simulation_shell(launch_file, log_file),
            ],
        )

    def set_simulation_busy(self, busy: bool) -> None:
        state = "disabled" if busy else "normal"
        for button in self.sim_buttons:
            button.configure(state=state)

    def run_simulation_action(self, action: str) -> None:
        try:
            endpoints = self.endpoints()
            user = self.user.get().strip()
            if not user:
                raise ValueError("User is required")
        except ValueError as exc:
            self.log_message(str(exc), "ERROR")
            messagebox.showerror("Jetson connection", str(exc))
            return

        self.set_simulation_busy(True)
        self.sim_status.set(f"{action.capitalize()} requested; locating Jetson ...")
        endpoint_list = ", ".join(f"{host}:{port}" for host, port in endpoints)
        self.log_message(f"{action.capitalize()} requested. Probing {endpoint_list}.")
        worker = threading.Thread(
            target=self._simulation_worker,
            args=(action, endpoints, user),
            daemon=True,
        )
        worker.start()

    def _simulation_worker(
        self,
        action: str,
        endpoints: list[tuple[str, int]],
        user: str,
    ) -> None:
        started_at = time.monotonic()
        try:
            endpoint = self.find_reachable_endpoint(endpoints)
            if endpoint is None:
                raise ConnectionError(
                    "Jetson is not reachable. Check the USB/Wi-Fi link."
                )
            host, port = endpoint
            target = f"{user}@{host}"
            self.log_message(f"Jetson reachable at {host}:{port}; using {target}.")
            if action == "start":
                message = self._start_simulation(target, port, host)
            elif action == "stop":
                message = self._stop_simulation(target, port)
            else:
                message = self._simulation_status(target, port, host)
        except (
            ConnectionError,
            RuntimeError,
            subprocess.TimeoutExpired,
            OSError,
        ) as exc:
            elapsed = time.monotonic() - started_at
            self.log_message(f"{action.capitalize()} failed after {elapsed:.1f}s: {exc}", "ERROR")
            self.ui_events.put(
                ("finish", f"Simulation action failed: {exc}", True)
            )
        else:
            elapsed = time.monotonic() - started_at
            self.log_message(f"{message} Completed in {elapsed:.1f}s.", "OK")
            self.ui_events.put(("finish", message, False))

    def _start_simulation(self, target: str, port: int, host: str) -> str:
        self.cleanup_regenerable_simulation_state(target, port)
        self.disk_preflight(target, port)
        world_check = self.run_ssh(
            target,
            port,
            ["test", "-r", SIM_WORLD],
            timeout=8,
            check=False,
        )
        if world_check.returncode:
            raise RuntimeError(
                "Dense v3 world is missing on Jetson: " + SIM_WORLD
            )
        states = self.service_states(target, port)
        self.log_message(f"Initial service states: {states}.")
        if all(state == "active" for state in states.values()) and self.sensor_topics_ready(
            target, port
        ):
            self.log_message("All required D435i topics are already available.")
            return f"D435i simulation is already running on {host} (ROS domain {SIM_DOMAIN_ID})"

        self.log_message("Stopping stale units and clearing previous failure state.")
        self.stop_units(target, port)
        self.cleanup_regenerable_simulation_state(target, port)
        self.run_ssh(target, port, ["systemctl", "--user", "reset-failed", *SIM_UNITS], check=False)
        try:
            self.log_message(f"Starting {GAZEBO_UNIT}.")
            self.start_unit(target, port, GAZEBO_UNIT, "gz_sim.launch.py")
            time.sleep(1)
            self.log_message(f"Starting {BRIDGE_UNIT}.")
            self.start_unit(target, port, BRIDGE_UNIT, "gz_bridge_portable.launch.py")
        except Exception:
            self.log_message("Unit startup failed; stopping the partial launch.", "WARN")
            self.stop_units(target, port)
            raise

        previous_states = None
        topics_wait_logged = False
        for _ in range(30):
            states = self.service_states(target, port)
            if states != previous_states:
                self.log_message(f"Service states: {states}.")
                previous_states = states
            if all(state == "active" for state in states.values()):
                if self.sensor_topics_ready(target, port):
                    self.log_message(
                        f"All {len(EXPECTED_SIM_TOPICS)} required D435i topics are ready."
                    )
                    return (
                        f"D435i simulation is running on {host} "
                        f"(ROS domain {SIM_DOMAIN_ID})"
                    )
                if not topics_wait_logged:
                    self.log_message("Services active; waiting for D435i topics.")
                    topics_wait_logged = True
            if any(state == "failed" for state in states.values()):
                break
            time.sleep(0.5)

        logs = self.service_logs(target, port)
        self.log_message(f"Recent Jetson service log:\n{logs}", "ERROR")
        self.stop_units(target, port)
        raise RuntimeError(f"services or D435i topics did not become ready: {states}\n{logs}")

    def _stop_simulation(self, target: str, port: int) -> str:
        self.log_message(f"Stopping {BRIDGE_UNIT}, then {GAZEBO_UNIT}.")
        self.stop_units(target, port)
        self.cleanup_regenerable_simulation_state(target, port)
        states = self.service_states(target, port)
        self.log_message(f"Final service states: {states}.")
        if any(state == "active" for state in states.values()):
            raise RuntimeError(f"services are still active: {states}")
        return "D435i simulation stopped"

    def _simulation_status(self, target: str, port: int, host: str) -> str:
        states = self.service_states(target, port)
        self.log_message(f"Service status refresh: {states}.")
        if all(state == "active" for state in states.values()):
            if not self.sensor_topics_ready(target, port):
                self.log_message("Services active, but required D435i topics are incomplete.", "WARN")
                return f"{host}: services active; waiting for D435i topics"
            self.log_message("Required D435i topics are available.")
            return f"D435i simulation is running on {host} (ROS domain {SIM_DOMAIN_ID})"
        if all(state in {"inactive", "unknown"} for state in states.values()):
            return "D435i simulation is stopped"
        return f"D435i simulation is incomplete: {states}"

    def _finish_simulation_action(self, message: str, failed: bool) -> None:
        self.set_simulation_busy(False)
        self.sim_status.set(message)
        if failed:
            messagebox.showerror("D435i simulation", message)

    def start_simulation(self) -> None:
        self.run_simulation_action("start")

    def stop_simulation(self) -> None:
        self.run_simulation_action("stop")

    def refresh_simulation_status(self) -> None:
        self.run_simulation_action("status")

    def close_window(self) -> None:
        """Clean the remote simulation before closing the Tk GUI."""
        if any(button.cget("state") == "disabled" for button in self.sim_buttons):
            self.status.set("An action is still running; close cleanup will follow it.")
            self.after(500, self.close_window)
            return
        try:
            endpoints = self.endpoints()
            user = self.user.get().strip()
            if not user:
                raise ValueError("User is required")
        except (ValueError, OSError):
            self.destroy()
            return

        self.set_simulation_busy(True)
        self.status.set("Closing GUI; cleaning Jetson simulation ...")
        self.log_message("GUI close requested; cleaning the simulation state.")

        def worker() -> None:
            try:
                endpoint = self.find_reachable_endpoint(endpoints)
                if endpoint is not None:
                    host, port = endpoint
                    self._stop_simulation(f"{user}@{host}", port)
                    self.log_message("GUI close cleanup completed.", "OK")
                else:
                    self.log_message(
                        "Jetson was unreachable; GUI closed without remote cleanup.",
                        "WARN",
                    )
            except Exception as exc:
                self.log_message(f"GUI close cleanup incomplete: {exc}", "WARN")
            finally:
                self.after(0, self.destroy)

        threading.Thread(target=worker, daemon=True).start()

    def check_connection(self) -> None:
        try:
            endpoints = self.endpoints()
        except ValueError as exc:
            messagebox.showerror("Invalid connection settings", str(exc))
            return

        self.status.set("Checking Jetson addresses ...")
        checked = ", ".join(f"{host}:{port}" for host, port in endpoints)
        self.log_message(f"Checking SSH reachability: {checked}.")
        self.update_idletasks()
        for host, port in endpoints:
            try:
                with socket.create_connection((host, port), timeout=2):
                    self.status.set(f"SSH port is reachable: {host}:{port}")
                    self.log_message(f"SSH port reachable at {host}:{port}.", "OK")
                    return
            except OSError:
                continue
        self.status.set(f"Cannot reach any configured Jetson address: {checked}")
        self.log_message(f"No configured SSH endpoint responded: {checked}.", "ERROR")

    def open_ssh_terminal(self) -> None:
        try:
            endpoint = self.find_reachable_endpoint()
        except ValueError as exc:
            messagebox.showerror("Invalid connection settings", str(exc))
            return
        if endpoint is None:
            self.status.set("Jetson is not reachable. Check the USB/Wi-Fi link and try again.")
            messagebox.showerror(
                "Jetson not reachable",
                "No configured SSH address responded. Reconnect the network, then press Reconnect SSH again.",
            )
            return

        host, port = endpoint
        user = self.user.get().strip()
        if not user:
            messagebox.showerror("Invalid connection settings", "User is required")
            return

        target = f"{user}@{host}"
        self.log_message(f"Opening an interactive SSH terminal for {target}:{port}.")
        if platform.system() != "Darwin":
            messagebox.showinfo("SSH command", f"Run: ssh -p {self.port.get().strip()} {target}")
            return

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
            "tell application \"Terminal\"",
            f"do script {json.dumps(terminal_script)}",
            "activate",
            "end tell",
        ]
        result = subprocess.run(["osascript", "-e", "\n".join(osa)], capture_output=True, text=True)
        if result.returncode:
            self.log_message(
                result.stderr.strip() or "osascript failed to open Terminal.",
                "ERROR",
            )
            messagebox.showerror("Could not open Terminal", result.stderr.strip() or "osascript failed")
        else:
            self.status.set(f"Connected to {target}; a fresh SSH terminal was opened")
            self.log_message(f"Interactive SSH terminal opened for {target}.", "OK")


if __name__ == "__main__":
    if shutil.which("python3") is None:
        raise SystemExit("python3 is required")
    JetsonConnectionApp().mainloop()
