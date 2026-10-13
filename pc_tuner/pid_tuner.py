from __future__ import annotations

import csv
import queue
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - depends on user environment
    list_ports = None

from plot_canvas import PlotCanvas
from telemetry import Telemetry, parse_telemetry
from transport import SerialWorker, SimulatedDrive


MAX_POINTS = 500


class PidTunerApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Dual-Wheel PID Tuner")
        self.geometry("1120x760")
        self.minsize(960, 660)
        self.inbox: queue.Queue[str] = queue.Queue()
        self.link: SerialWorker | SimulatedDrive | None = None
        self.samples: list[Telemetry] = []
        self.log_lines: list[str] = []
        self.active_command: str | None = None
        self.last_heartbeat = 0.0
        self._build_ui()
        self._refresh_ports()
        self.after(50, self._poll_messages)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=12)
        root.pack(fill="both", expand=True)
        top = ttk.Frame(root)
        top.pack(fill="x")
        ttk.Label(top, text="Port").pack(side="left")
        self.port_box = ttk.Combobox(top, width=16, state="readonly")
        self.port_box.pack(side="left", padx=6)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")
        ttk.Button(top, text="Connect", command=self._connect).pack(side="left", padx=6)
        ttk.Button(top, text="Sim", command=self._start_sim).pack(side="left")
        ttk.Button(top, text="Disconnect", command=self._disconnect).pack(side="left", padx=6)
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side="right")

        body = ttk.Frame(root)
        body.pack(fill="both", expand=True, pady=10)
        controls = ttk.Frame(body, width=272)
        controls.pack(side="left", fill="y", padx=(0, 12))
        controls.pack_propagate(False)
        self._build_controls(controls)
        chart = ttk.Frame(body)
        chart.pack(side="left", fill="both", expand=True)
        self.plot = PlotCanvas(chart)
        self.plot.pack(fill="both", expand=True)
        ttk.Button(chart, text="Save CSV", command=self._save_csv).pack(anchor="e", pady=(8, 0))
        self.log = tk.Text(root, height=9, wrap="none")
        self.log.pack(fill="x")

    def _build_controls(self, parent: ttk.Frame) -> None:
        wheel = ttk.LabelFrame(parent, text="Wheel RPM")
        wheel.pack(fill="x", pady=(0, 10))
        self.left_rpm = tk.StringVar(value="80")
        self.right_rpm = tk.StringVar(value="80")
        self._entry(wheel, "Left", self.left_rpm)
        self._entry(wheel, "Right", self.right_rpm)
        ttk.Button(wheel, text="Send Wheel RPM", command=self._send_wheel).pack(fill="x", pady=6)

        cmdvel = ttk.LabelFrame(parent, text="Differential Command")
        cmdvel.pack(fill="x", pady=(0, 10))
        self.linear = tk.StringVar(value="0.12")
        self.angular = tk.StringVar(value="0.0")
        self._entry(cmdvel, "Linear m/s", self.linear)
        self._entry(cmdvel, "Angular rad/s", self.angular)
        ttk.Button(cmdvel, text="Send v / w", command=self._send_cmdvel).pack(fill="x", pady=6)
        row = ttk.Frame(cmdvel)
        row.pack(fill="x")
        ttk.Button(row, text="Stop", command=self._stop_motion).pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="Zero Odom", command=lambda: self._send("ZERO")).pack(side="left", fill="x", expand=True, padx=(6, 0))

        pid = ttk.LabelFrame(parent, text="Speed PID")
        pid.pack(fill="x", pady=(0, 10))
        self.kp, self.ki, self.kd = tk.StringVar(value="1.2"), tk.StringVar(value="0.35"), tk.StringVar(value="0.01")
        self._entry(pid, "Kp", self.kp)
        self._entry(pid, "Ki", self.ki)
        self._entry(pid, "Kd", self.kd)
        ttk.Button(pid, text="Apply Both", command=lambda: self._send_pid("SPEED")).pack(fill="x", pady=(6, 0))
        row = ttk.Frame(pid)
        row.pack(fill="x", pady=(6, 0))
        ttk.Button(row, text="Apply Left", command=lambda: self._send_pid("LEFT")).pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="Apply Right", command=lambda: self._send_pid("RIGHT")).pack(side="left", fill="x", expand=True, padx=(6, 0))

        config = ttk.LabelFrame(parent, text="Vehicle Config")
        config.pack(fill="x")
        self.cpr, self.diameter, self.base = tk.StringVar(value="1320"), tk.StringVar(value="0.065"), tk.StringVar(value="0.155")
        self._entry(config, "Ticks/rev", self.cpr)
        self._entry(config, "Wheel diameter m", self.diameter)
        self._entry(config, "Wheel base m", self.base)
        ttk.Button(config, text="Send Config", command=self._send_config).pack(fill="x", pady=6)

    @staticmethod
    def _entry(parent: ttk.Frame, label: str, variable: tk.StringVar) -> None:
        ttk.Label(parent, text=label).pack(anchor="w")
        ttk.Entry(parent, textvariable=variable).pack(fill="x")

    def _refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()] if list_ports else []
        ports = ports or [f"COM{index}" for index in range(1, 13)]
        self.port_box["values"] = ports
        self.port_box.set(ports[0])

    def _connect(self) -> None:
        self._disconnect()
        try:
            self.link = SerialWorker(self.port_box.get(), self.inbox)
            self.status_var.set(f"Connected: {self.port_box.get()}")
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))

    def _start_sim(self) -> None:
        self._disconnect()
        self.link = SimulatedDrive(self.inbox)
        self.status_var.set("Simulation")

    def _disconnect(self) -> None:
        self.active_command = None
        if self.link:
            self.link.close()
            self.link = None
        self.status_var.set("Disconnected")

    def _send(self, command: str) -> None:
        self._append_log(f"> {command}")
        if self.link:
            self.link.send(command)

    def _activate_motion(self, command: str) -> None:
        self.active_command = command
        self.last_heartbeat = time.monotonic()
        self._send(command)

    def _stop_motion(self) -> None:
        self.active_command = None
        self._send("STOP")

    def _send_wheel(self) -> None:
        self._activate_motion(f"WHEEL {self.left_rpm.get()} {self.right_rpm.get()}")

    def _send_cmdvel(self) -> None:
        self._activate_motion(f"CMDVEL {self.linear.get()} {self.angular.get()}")

    def _send_pid(self, target: str) -> None:
        self._send(f"PID {target} {self.kp.get()} {self.ki.get()} {self.kd.get()}")

    def _send_config(self) -> None:
        self._send(f"CONFIG CPR {self.cpr.get()}")
        self._send(f"CONFIG WHEEL_DIAM {self.diameter.get()}")
        self._send(f"CONFIG WHEEL_BASE {self.base.get()}")

    def _poll_messages(self) -> None:
        while True:
            try:
                line = self.inbox.get_nowait()
            except queue.Empty:
                break
            sample = parse_telemetry(line)
            if sample:
                self.samples = (self.samples + [sample])[-MAX_POINTS:]
            else:
                self._append_log(line)
        if self.active_command and self.link and time.monotonic() - self.last_heartbeat >= 0.2:
            self.link.send(self.active_command)
            self.last_heartbeat = time.monotonic()
        self.plot.draw_plot(self.samples)
        self.after(50, self._poll_messages)

    def _append_log(self, line: str) -> None:
        self.log_lines = (self.log_lines + [line])[-200:]
        self.log.delete("1.0", "end")
        self.log.insert("end", "\n".join(self.log_lines) + "\n")
        self.log.see("end")

    def _save_csv(self) -> None:
        if not self.samples:
            messagebox.showinfo("Save CSV", "No telemetry data yet")
            return
        default = Path.cwd() / "logs" / f"dual_wheel_pid_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        default.parent.mkdir(exist_ok=True)
        filename = filedialog.asksaveasfilename(initialfile=default.name, initialdir=str(default.parent), defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if filename:
            with open(filename, "w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(Telemetry.__dataclass_fields__.keys())
                writer.writerows(vars(item).values() for item in self.samples)
            messagebox.showinfo("Save CSV", f"Saved {filename}")

    def _on_close(self) -> None:
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    PidTunerApp().mainloop()
