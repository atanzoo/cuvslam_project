#!/usr/bin/env python3
"""macOS Tk GUI for guided real-D435i IMU axis phases.

The GUI starts the Jetson-side real-camera container, launches one manually
controlled phase at a time, renders live gyro values, and downloads each JSON
result into the local R1.5 log directory.
"""

from __future__ import annotations

import json
import platform
import queue
import shlex
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk


REMOTE_SCRIPT = (
    "/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/"
    "run_real_d435i_r1p5_axis.sh"
)
REMOTE_DATA_DIR = "/home/tseng/isaac_ros_data/real_robot/cuvslam/experiments/real_d435i_r1p5_gui"
PHASES = {
    "optical_x_up": ("X 軸：鏡頭向上抬", "把鏡頭向上抬 60–90°，約 1–2 秒完成"),
    "optical_x_down": ("X 軸反向：鏡頭向下壓", "把鏡頭向下壓回原位，約 1–2 秒完成"),
    "optical_y_left": ("Y 軸：鏡頭向左轉", "像搖頭一樣向左轉 60–90°，約 1–2 秒完成"),
    "optical_y_right": ("Y 軸反向：鏡頭向右轉", "向右轉回原位，約 1–2 秒完成"),
    "optical_z_clockwise": ("Z 軸：畫面順時針扭轉", "在畫面平面內順時針扭轉 60–90°"),
    "optical_z_counterclockwise": ("Z 軸反向：畫面逆時針扭轉", "逆時針扭轉回原位"),
}


class RealD435iAxisApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Real D435i · R1.5 三軸 IMU 測試")
        self.geometry("1060x720")
        self.minsize(900, 620)
        self.configure(background="#1f2937")

        self.host = tk.StringVar(value="192.168.55.1")
        self.user = tk.StringVar(value="tseng")
        self.port = tk.StringVar(value="22")
        self.duration = tk.StringVar(value="10")
        self.status = tk.StringVar(value="尚未啟動測試容器")
        self.phase_text = tk.StringVar(value="請先按「啟動實機測試」")
        self.live_text = tk.StringVar(value="gx 0.000 · gy 0.000 · gz 0.000 rad/s")
        self.peak_text = tk.StringVar(value="peak 0.000 · dominant —")
        self.elapsed_text = tk.StringVar(value="elapsed 0.0 s")
        self.ui_events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.phase_process: subprocess.Popen[str] | None = None
        self.current_phase = ""
        self.current_output_name = ""
        self.phase_buttons: list[tk.Button] = []
        self.live_bars: dict[str, int] = {}
        self.result_text = None
        self.log_text = None

        self._build_ui()
        self.after(100, self._drain_events)
        self.protocol("WM_DELETE_WINDOW", self._close)
        if platform.system() == "Darwin":
            self.after_idle(self._bring_to_front)

    def _bring_to_front(self) -> None:
        self.deiconify()
        self.lift()
        self.focus_force()

    def _build_ui(self) -> None:
        bg = "#1f2937"
        fg = "#f8fafc"
        muted = "#cbd5e1"
        panel = "#111827"

        # Use classic Tk widgets with explicit colors.  The bundled Xcode
        # Python/Tk can render native ttk widgets as an empty dark surface on
        # some macOS dark-mode configurations.
        root = tk.Frame(self, bg=bg, padx=18, pady=18)
        root.grid(row=0, column=0, sticky="nsew")
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        root.rowconfigure(3, weight=1)
        root.columnconfigure(1, weight=1)
        root.columnconfigure(2, weight=1)

        tk.Label(root, text="Real D435i · R1.5 三軸 IMU 測試", bg=bg, fg=fg,
                 font=("Helvetica", 18, "bold")).grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 12)
        )

        connection = tk.LabelFrame(root, text="Jetson 連線", bg=bg, fg=fg,
                                    padx=10, pady=10, bd=1, relief="groove")
        connection.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(0, 10))
        for index, (label, variable, width) in enumerate(
            (("Host", self.host, 18), ("User", self.user, 10), ("Port", self.port, 6))
        ):
            tk.Label(connection, text=label, bg=bg, fg=fg).grid(row=0, column=index * 2, padx=(0, 5))
            tk.Entry(connection, textvariable=variable, width=width, bg="#ffffff", fg="#111827",
                     insertbackground=fg, relief="solid", bd=1).grid(
                row=0, column=index * 2 + 1, padx=(0, 14)
            )
        tk.Label(connection, text="每段秒數", bg=bg, fg=fg).grid(row=0, column=6, padx=(0, 5))
        tk.Entry(connection, textvariable=self.duration, width=6, bg="#ffffff", fg="#111827",
                 insertbackground=fg, relief="solid", bd=1).grid(row=0, column=7, padx=(0, 12))
        tk.Button(
            connection, text="啟動實機測試", command=self.start_rig, bg="#2563eb", fg="#ffffff",
            activebackground="#1d4ed8", activeforeground="#ffffff", relief="flat", padx=12, pady=6
        ).grid(row=0, column=8, padx=(0, 6))
        tk.Button(connection, text="停止", command=self.stop_rig, bg="#e2e8f0", fg=fg,
                  activebackground="#cbd5e1", relief="flat", padx=12, pady=6).grid(row=0, column=9)

        tk.Label(root, textvariable=self.status, bg=bg, fg=muted).grid(
            row=2, column=0, columnspan=3, sticky="w", pady=(0, 10)
        )

        phase_panel = tk.LabelFrame(root, text="按鈕式 phase 測試", bg=bg, fg=fg,
                                    padx=12, pady=12, bd=1, relief="groove")
        phase_panel.grid(row=3, column=0, sticky="nsw", padx=(0, 12))
        tk.Label(phase_panel, textvariable=self.phase_text, bg=bg, fg=fg, wraplength=260,
                 justify="left", anchor="w").grid(
            row=0, column=0, sticky="w", pady=(0, 10)
        )
        for row, (phase, (title, _)) in enumerate(PHASES.items(), start=1):
            button = tk.Button(
                phase_panel,
                text=title,
                command=lambda name=phase: self.start_phase(name),
                width=29,
                bg="#e2e8f0", fg=fg, activebackground="#cbd5e1", relief="flat", pady=5,
            )
            button.grid(row=row, column=0, sticky="ew", pady=3)
            self.phase_buttons.append(button)

        live_panel = tk.LabelFrame(root, text="即時 IMU", bg=bg, fg=fg,
                                   padx=12, pady=12, bd=1, relief="groove")
        live_panel.grid(row=3, column=1, sticky="nsew")
        live_panel.columnconfigure(0, weight=1)
        tk.Label(live_panel, textvariable=self.live_text, bg=bg, fg=fg, font=("Menlo", 13)).grid(
            row=0, column=0, sticky="w", pady=(0, 4)
        )
        tk.Label(live_panel, textvariable=self.peak_text, bg=bg, fg=fg, font=("Menlo", 11)).grid(
            row=1, column=0, sticky="w", pady=(0, 4)
        )
        tk.Label(live_panel, textvariable=self.elapsed_text, bg=bg, fg=muted).grid(row=2, column=0, sticky="w")
        self.bar_canvas = tk.Canvas(
            live_panel, height=180, background="#111827", highlightthickness=0
        )
        self.bar_canvas.grid(row=3, column=0, sticky="ew", pady=(14, 10))
        self._draw_bars({"gx": 0.0, "gy": 0.0, "gz": 0.0})

        result_panel = tk.LabelFrame(root, text="目前結果", bg=bg, fg=fg,
                                     padx=10, pady=10, bd=1, relief="groove")
        result_panel.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        result_panel.columnconfigure(0, weight=1)
        self.result_text = tk.Text(result_panel, height=6, width=80, state="disabled", font=("Menlo", 10),
                                   bg=panel, fg=fg, insertbackground=fg, relief="solid", bd=1)
        self.result_text.grid(row=0, column=0, sticky="ew")

        log_panel = tk.LabelFrame(root, text="工作記錄", bg=bg, fg=fg,
                                  padx=10, pady=10, bd=1, relief="groove")
        log_panel.grid(row=3, column=2, rowspan=2, sticky="nsew", padx=(12, 0))
        root.columnconfigure(2, weight=1)
        self.log_text = tk.Text(log_panel, width=36, height=22, state="disabled", font=("Menlo", 9),
                                bg=panel, fg=fg, insertbackground=fg, relief="solid", bd=1)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_panel.rowconfigure(0, weight=1)
        log_panel.columnconfigure(0, weight=1)

    def _ssh_command(self, remote_args: list[str]) -> list[str]:
        target = f"{self.user.get().strip()}@{self.host.get().strip()}"
        return [
            "ssh", "-p", self.port.get().strip(), "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=15",
            "-o", "ServerAliveCountMax=3", "-o", "ControlMaster=no",
            "-o", "ControlPath=none", "-o", "StrictHostKeyChecking=accept-new",
            target, shlex.join(remote_args),
        ]

    def _run_background(self, remote_args: list[str], event: str) -> None:
        def worker() -> None:
            try:
                result = subprocess.run(
                    self._ssh_command(remote_args), capture_output=True, text=True, timeout=90
                )
                output = (result.stdout + result.stderr).strip()
                self.ui_events.put((event, (result.returncode, output)))
            except Exception as exc:  # noqa: BLE001 - shown in the GUI
                self.ui_events.put((event, (1, str(exc))))

        threading.Thread(target=worker, daemon=True).start()

    def start_rig(self) -> None:
        self.status.set("正在啟動 Jetson D435i + cuVSLAM...")
        self._run_background(["bash", REMOTE_SCRIPT, "start"], "rig_start")

    def stop_rig(self) -> None:
        if self.phase_process is not None:
            self.phase_process.terminate()
            self.phase_process = None
        self._run_background(["bash", REMOTE_SCRIPT, "stop"], "rig_stop")

    def start_phase(self, phase: str) -> None:
        if self.phase_process is not None:
            messagebox.showinfo("測試進行中", "目前 phase 尚未完成，請先等待結果。")
            return
        try:
            duration = float(self.duration.get())
            if not 3.0 <= duration <= 60.0:
                raise ValueError
        except ValueError:
            messagebox.showerror("秒數錯誤", "每段秒數請填 3 到 60。")
            return

        self.current_phase = phase
        self.current_output_name = f"{time.strftime('%Y%m%d_%H%M%S')}_{phase}.json"
        title, instruction = PHASES[phase]
        self.phase_text.set(f"{title}\n{instruction}\n\n現在請按動作，收樣中...")
        self.status.set(f"收集 {title}，{duration:.1f} 秒")
        self._set_phase_buttons("disabled")
        command = [
            "bash", REMOTE_SCRIPT, "phase", phase, f"{duration:g}", self.current_output_name
        ]
        self.phase_process = subprocess.Popen(
            self._ssh_command(command),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        threading.Thread(target=self._read_phase_output, args=(self.phase_process,), daemon=True).start()

    def _read_phase_output(self, process: subprocess.Popen[str]) -> None:
        assert process.stdout is not None
        for raw_line in process.stdout:
            self.ui_events.put(("phase_line", raw_line.strip()))
        return_code = process.wait()
        self.ui_events.put(("phase_done", return_code))

    def _drain_events(self) -> None:
        try:
            while True:
                event, payload = self.ui_events.get_nowait()
                if event == "phase_line":
                    self._handle_phase_line(str(payload))
                elif event == "phase_done":
                    self.phase_process = None
                    self._set_phase_buttons("normal")
                    self.status.set("phase 完成；可選擇下一個方向")
                    self._download_result()
                elif event == "rig_start":
                    code, output = payload
                    self.status.set("實機容器已啟動" if code == 0 else "啟動失敗")
                    self.log(output or self.status.get())
                elif event == "rig_stop":
                    code, output = payload
                    self.status.set("實機容器已停止" if code == 0 else "停止完成或容器不存在")
                    self.log(output or self.status.get())
        except queue.Empty:
            pass
        self.after(100, self._drain_events)

    def _handle_phase_line(self, line: str) -> None:
        if line.startswith("LIVE "):
            try:
                data = json.loads(line[5:])
                gyro = data["gyro_rad_s"]
                peaks = data["peak_abs_gyro_rad_s"]
                self.live_text.set(
                    f"gx {gyro[0]:+.3f} · gy {gyro[1]:+.3f} · gz {gyro[2]:+.3f} rad/s"
                )
                self.peak_text.set(
                    f"peak {max(peaks):.3f} · dominant axis {data['dominant_axis']}"
                )
                self.elapsed_text.set(f"elapsed {data['elapsed_s']:.1f} s")
                self._draw_bars({"gx": gyro[0], "gy": gyro[1], "gz": gyro[2]})
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                self.log(line)
            return
        if line.startswith("PHASE "):
            self.log(line)
            return
        if line.startswith("{"):
            try:
                result = json.loads(line)
                self._show_result(result)
                self.log(json.dumps(result, ensure_ascii=False))
                return
            except json.JSONDecodeError:
                pass
        if line:
            self.log(line)

    def _show_result(self, result: dict[str, object]) -> None:
        if self.result_text is None:
            return
        summary = (
            f"{result.get('label')}  samples={result.get('samples')}  "
            f"dominant={result.get('dominant_axis')}\n"
            f"integrated={result.get('integrated_gyro_rad')}\n"
            f"peak={result.get('peak_abs_gyro_rad_s')}\n\n"
        )
        self.result_text.configure(state="normal")
        self.result_text.insert("end", summary)
        self.result_text.see("end")
        self.result_text.configure(state="disabled")

    def _download_result(self) -> None:
        if not self.current_output_name:
            return
        project_root = Path(__file__).resolve().parents[3]
        local_dir = project_root / "real_robot" / "cuvslam" / "evidence" / "logs" / "real_d435i_r1p5_gui"
        local_dir.mkdir(parents=True, exist_ok=True)
        remote = f"{self.user.get().strip()}@{self.host.get().strip()}:{REMOTE_DATA_DIR}/{self.current_output_name}"
        command = [
            "scp", "-P", self.port.get().strip(), "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=8", remote, str(local_dir / self.current_output_name),
        ]
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=30)
            self.log(
                f"已下載 {local_dir / self.current_output_name}"
                if result.returncode == 0
                else (result.stderr.strip() or "下載結果失敗")
            )
        except Exception as exc:  # noqa: BLE001 - shown in the GUI
            self.log(f"下載結果失敗：{exc}")

    def _draw_bars(self, values: dict[str, float]) -> None:
        canvas = self.bar_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 460)
        center = width / 2
        scale = max((width / 2 - 70) / 3.0, 50.0)
        for row, axis in enumerate(("gx", "gy", "gz")):
            y = 32 + row * 48
            value = float(values.get(axis, 0.0))
            extent = min(abs(value) * scale, width / 2 - 70)
            canvas.create_text(20, y, text=axis, fill="#e5e7eb", font=("Menlo", 12), anchor="w")
            canvas.create_line(center, y - 14, center, y + 14, fill="#94a3b8")
            color = "#38bdf8" if value >= 0 else "#fb7185"
            canvas.create_rectangle(
                center,
                y - 9,
                center + (extent if value >= 0 else -extent),
                y + 9,
                fill=color,
                outline="",
            )
            canvas.create_text(width - 12, y, text=f"{value:+.3f}", fill="#e5e7eb", font=("Menlo", 10), anchor="e")

    def _set_phase_buttons(self, state: str) -> None:
        for button in self.phase_buttons:
            button.configure(state=state)

    def log(self, message: str) -> None:
        if self.log_text is None:
            return
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _close(self) -> None:
        if self.phase_process is not None:
            self.phase_process.terminate()
        self.destroy()


if __name__ == "__main__":
    RealD435iAxisApp().mainloop()
