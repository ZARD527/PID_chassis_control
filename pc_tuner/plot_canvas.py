from __future__ import annotations

import math
import tkinter as tk

from telemetry import Telemetry


class PlotCanvas(tk.Canvas):
    def __init__(self, master: tk.Misc) -> None:
        super().__init__(master, height=340, background="#101820", highlightthickness=0, width=820)

    def draw_plot(self, samples: list[Telemetry]) -> None:
        self.delete("all")
        width, height = max(10, self.winfo_width()), max(10, self.winfo_height())
        pad_l, pad_r, pad_t, pad_b = 48, 18, 24, 32
        plot_w, plot_h = width - pad_l - pad_r, height - pad_t - pad_b
        self.create_rectangle(pad_l, pad_t, width - pad_r, height - pad_b, outline="#38505d")
        for index in range(5):
            y = pad_t + index * plot_h / 4
            self.create_line(pad_l, y, width - pad_r, y, fill="#20323c")
        if not samples:
            self.create_text(width / 2, height / 2, text="waiting for data", fill="#9fb3bf")
            return

        series = [
            ([item.target_left for item in samples], "#ffcc66", "left target"),
            ([item.rpm_left for item in samples], "#4fd1c5", "left actual"),
            ([item.target_right for item in samples], "#e788f5", "right target"),
            ([item.rpm_right for item in samples], "#6da8ff", "right actual"),
        ]
        values = [value for points, _, _ in series for value in points]
        low, high = min(values), max(values)
        if math.isclose(low, high):
            low, high = low - 1.0, high + 1.0
        margin = (high - low) * 0.12
        low, high = low - margin, high + margin

        def point(index: int, value: float) -> tuple[float, float]:
            x = pad_l + index / max(1, len(samples) - 1) * plot_w
            y = pad_t + (high - value) / (high - low) * plot_h
            return x, y

        for points, color, _ in series:
            self._draw_line([point(index, value) for index, value in enumerate(points)], color)
        self.create_text(pad_l, 12, text=f"{high:.1f} rpm", fill="#9fb3bf", anchor="w")
        self.create_text(pad_l, height - 12, text=f"{low:.1f} rpm", fill="#9fb3bf", anchor="w")
        for index, (_, color, label) in enumerate(series):
            self.create_text(145 + index * 132, 12, text=label, fill=color, anchor="w")

    def _draw_line(self, points: list[tuple[float, float]], color: str) -> None:
        if len(points) >= 2:
            self.create_line(*[axis for point in points for axis in point], fill=color, width=2, smooth=True)
