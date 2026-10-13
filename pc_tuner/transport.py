from __future__ import annotations

import math
import queue
import random
import threading
import time

try:
    import serial
except ImportError:  # pragma: no cover - depends on the local environment
    serial = None


BAUDRATE = 115200


class SerialWorker:
    def __init__(self, port: str, inbox: queue.Queue[str]) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed; run pip install -r requirements.txt")
        self._inbox = inbox
        self._serial = serial.Serial(port, BAUDRATE, timeout=0.05)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        while self._running:
            try:
                raw = self._serial.readline()
                if raw:
                    self._inbox.put(raw.decode("utf-8", errors="replace").strip())
            except Exception as exc:
                self._inbox.put(f"ERR serial read failed: {exc}")
                break

    def send(self, command: str) -> None:
        if self._serial.is_open:
            self._serial.write((command.strip() + "\n").encode("ascii", errors="ignore"))

    def close(self) -> None:
        self._running = False
        try:
            self._serial.close()
        except Exception:
            pass


class SimulatedDrive:
    """Small two-wheel plant used by the same commands as the Arduino firmware."""

    def __init__(self, inbox: queue.Queue[str]) -> None:
        self._inbox = inbox
        self._mode = "STOP"
        self._target_left = 0.0
        self._target_right = 0.0
        self._rpm_left = 0.0
        self._rpm_right = 0.0
        self._pwm_left = 0.0
        self._pwm_right = 0.0
        self._integral_left = self._integral_right = 0.0
        self._previous_left = self._previous_right = 0.0
        self._gains_left = [1.2, 0.35, 0.01]
        self._gains_right = [1.2, 0.35, 0.01]
        self._x = self._y = self._theta = 0.0
        self._running = True
        self._start_time = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def send(self, command: str) -> None:
        parts = command.strip().upper().split()
        if not parts:
            return
        try:
            if parts[0] in {"STOP", "BRAKE", "FREE"}:
                self._mode = "STOP"
                self._target_left = self._target_right = 0.0
            elif parts[0] == "WHEEL" and len(parts) == 3:
                self._target_left, self._target_right = float(parts[1]), float(parts[2])
                self._mode = "WHEEL"
            elif parts[0] == "CMDVEL" and len(parts) == 3:
                linear, angular = float(parts[1]), float(parts[2])
                wheel_base, wheel_diameter = 0.155, 0.065
                scale = 60.0 / (math.pi * wheel_diameter)
                self._target_left = (linear - angular * wheel_base / 2.0) * scale
                self._target_right = (linear + angular * wheel_base / 2.0) * scale
                self._mode = "CMDVEL"
            elif parts[0] == "PID" and len(parts) == 5:
                gains = [float(value) for value in parts[2:]]
                if parts[1] in {"SPEED", "BOTH"}:
                    self._gains_left = gains[:]
                    self._gains_right = gains[:]
                elif parts[1] == "LEFT":
                    self._gains_left = gains
                elif parts[1] == "RIGHT":
                    self._gains_right = gains
                else:
                    raise ValueError("PID target")
            elif parts[0] == "ZERO":
                self._x = self._y = self._theta = 0.0
            else:
                self._inbox.put(f"ERR unsupported simulation command: {command}")
                return
            self._inbox.put(f"OK sim {parts[0].lower()}")
        except ValueError:
            self._inbox.put(f"ERR invalid simulation command: {command}")

    def close(self) -> None:
        self._running = False

    def _loop(self) -> None:
        previous = time.time()
        while self._running:
            now = time.time()
            dt = max(0.001, now - previous)
            previous = now
            self._step(dt)
            self._inbox.put(self._telemetry(int((now - self._start_time) * 1000)))
            time.sleep(0.05)

    def _step_wheel(
        self, target: float, actual: float, integral: float, previous_error: float, gains: list[float], dt: float
    ) -> tuple[float, float, float, float]:
        error = target - actual
        integral = max(-500.0, min(500.0, integral + error * dt))
        derivative = (error - previous_error) / dt
        pwm = max(-255.0, min(255.0, gains[0] * error + gains[1] * integral + gains[2] * derivative))
        actual += ((pwm * 0.95 - actual) / 0.18) * dt + random.uniform(-0.5, 0.5)
        return actual, pwm, integral, error

    def _step(self, dt: float) -> None:
        left_target = self._target_left if self._mode != "STOP" else 0.0
        right_target = self._target_right if self._mode != "STOP" else 0.0
        self._rpm_left, self._pwm_left, self._integral_left, left_error = self._step_wheel(
            left_target, self._rpm_left, self._integral_left, self._previous_left, self._gains_left, dt
        )
        self._rpm_right, self._pwm_right, self._integral_right, right_error = self._step_wheel(
            right_target, self._rpm_right, self._integral_right, self._previous_right, self._gains_right, dt
        )
        self._previous_left, self._previous_right = left_error, right_error
        circumference, wheel_base = math.pi * 0.065, 0.155
        left_distance = self._rpm_left / 60.0 * circumference * dt
        right_distance = self._rpm_right / 60.0 * circumference * dt
        delta_theta = (right_distance - left_distance) / wheel_base
        distance = (left_distance + right_distance) / 2.0
        self._x += distance * math.cos(self._theta + delta_theta / 2.0)
        self._y += distance * math.sin(self._theta + delta_theta / 2.0)
        self._theta += delta_theta

    def _telemetry(self, elapsed_ms: int) -> str:
        circumference, wheel_base = math.pi * 0.065, 0.155
        linear = (self._rpm_left + self._rpm_right) * circumference / 120.0
        angular = (self._rpm_right - self._rpm_left) * circumference / (60.0 * wheel_base)
        return (
            f"DATA,t={elapsed_ms},mode={self._mode},target_l={self._target_left:.2f},rpm_l={self._rpm_left:.2f},"
            f"pwm_l={self._pwm_left:.1f},err_l={self._target_left - self._rpm_left:.2f},"
            f"target_r={self._target_right:.2f},rpm_r={self._rpm_right:.2f},"
            f"pwm_r={self._pwm_right:.1f},err_r={self._target_right - self._rpm_right:.2f},"
            f"x={self._x:.4f},y={self._y:.4f},theta={self._theta:.4f},v={linear:.3f},w={angular:.3f}"
        )
