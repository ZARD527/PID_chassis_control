from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Telemetry:
    t: float
    mode: str
    target_left: float
    rpm_left: float
    pwm_left: float
    error_left: float
    target_right: float
    rpm_right: float
    pwm_right: float
    error_right: float
    x: float
    y: float
    theta: float
    linear_velocity: float
    angular_velocity: float


def parse_telemetry(line: str) -> Telemetry | None:
    """Parse a DATA line emitted by RobotController without raising on bad serial data."""
    if not line.startswith("DATA,"):
        return None
    try:
        fields = {}
        for part in line.split(",")[1:]:
            if "=" in part:
                key, value = part.split("=", 1)
                fields[key.strip()] = value.strip()
        return Telemetry(
            t=float(fields.get("t", "0")),
            mode=fields.get("mode", "UNKNOWN"),
            target_left=float(fields.get("target_l", "0")),
            rpm_left=float(fields.get("rpm_l", "0")),
            pwm_left=float(fields.get("pwm_l", "0")),
            error_left=float(fields.get("err_l", "0")),
            target_right=float(fields.get("target_r", "0")),
            rpm_right=float(fields.get("rpm_r", "0")),
            pwm_right=float(fields.get("pwm_r", "0")),
            error_right=float(fields.get("err_r", "0")),
            x=float(fields.get("x", "0")),
            y=float(fields.get("y", "0")),
            theta=float(fields.get("theta", "0")),
            linear_velocity=float(fields.get("v", "0")),
            angular_velocity=float(fields.get("w", "0")),
        )
    except ValueError:
        return None
