#!/usr/bin/env python3
"""Render desired vs actual tool trajectory from a control CSV log as a GIF."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


AXES = {
    "xy": (0, 1, "X (m)", "Y (m)"),
    "xz": (0, 2, "X (m)", "Z (m)"),
    "yz": (1, 2, "Y (m)", "Z (m)"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="CSV log with tool_x/y/z and tool_ref_x/y/z columns")
    parser.add_argument("--out", type=Path, default=None, help="Output GIF path")
    parser.add_argument("--plane", choices=tuple(AXES), default="xz", help="2D projection plane")
    parser.add_argument("--fps", type=int, default=24, help="GIF frame rate")
    parser.add_argument("--max-frames", type=int, default=180, help="Maximum frames to render")
    parser.add_argument("--trail", type=int, default=180, help="Actual-path trail length in rendered frames")
    parser.add_argument("--title", default=None, help="Plot title")
    return parser.parse_args()


def read_log(path: Path) -> dict[str, list[float]]:
    required = ("time_s", "tool_x", "tool_y", "tool_z", "tool_ref_x", "tool_ref_y", "tool_ref_z")
    data = {key: [] for key in required}
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise RuntimeError(f"{path} is empty or has no CSV header")
        missing = [key for key in required if key not in reader.fieldnames]
        if missing:
          raise RuntimeError(f"{path} is missing required columns: {', '.join(missing)}")
        for row in reader:
            for key in required:
                data[key].append(float(row[key]))
    if not data["time_s"]:
        raise RuntimeError(f"{path} has no samples")
    return data


def sampled_indices(sample_count: int, max_frames: int) -> list[int]:
    if sample_count <= max_frames:
        return list(range(sample_count))
    stride = max(1, math.ceil(sample_count / max_frames))
    indices = list(range(0, sample_count, stride))
    if indices[-1] != sample_count - 1:
        indices.append(sample_count - 1)
    return indices


def bounds(values_a: list[float], values_b: list[float], pad_ratio: float = 0.08) -> tuple[float, float]:
    lo = min(min(values_a), min(values_b))
    hi = max(max(values_a), max(values_b))
    span = hi - lo
    if span < 1.0e-6:
        span = 1.0
    pad = span * pad_ratio
    return lo - pad, hi + pad


def render_gif(args: argparse.Namespace) -> Path:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation, PillowWriter
    except ImportError as exc:
        raise RuntimeError(
            "render_control_log_gif.py requires matplotlib and pillow. "
            "Update the conda env with: conda env update -f environment.yml --prune"
        ) from exc

    data = read_log(args.log)
    axis_a, axis_b, label_a, label_b = AXES[args.plane]
    suffixes = ("x", "y", "z")
    actual_a = data[f"tool_{suffixes[axis_a]}"]
    actual_b = data[f"tool_{suffixes[axis_b]}"]
    ref_a = data[f"tool_ref_{suffixes[axis_a]}"]
    ref_b = data[f"tool_ref_{suffixes[axis_b]}"]
    time_s = data["time_s"]
    indices = sampled_indices(len(time_s), max(2, args.max_frames))

    out_path = args.out
    if out_path is None:
        out_path = args.log.with_suffix(".gif")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, (ax_path, ax_err) = plt.subplots(1, 2, figsize=(10.5, 4.8), gridspec_kw={"width_ratios": [1.35, 1.0]})
    title = args.title or args.log.stem
    fig.suptitle(title)

    ax_path.set_xlabel(label_a)
    ax_path.set_ylabel(label_b)
    ax_path.set_aspect("equal", adjustable="box")
    ax_path.set_xlim(*bounds(actual_a, ref_a))
    ax_path.set_ylim(*bounds(actual_b, ref_b))
    ax_path.grid(True, alpha=0.25)
    ax_path.plot(ref_a, ref_b, color="#26c6da", linewidth=1.5, alpha=0.35, label="target full")
    target_line, = ax_path.plot([], [], color="#26c6da", linewidth=2.0, label="target")
    actual_line, = ax_path.plot([], [], color="#ff8f33", linewidth=2.3, label="actual")
    target_marker, = ax_path.plot([], [], "o", color="#26c6da", markersize=5)
    actual_marker, = ax_path.plot([], [], "o", color="#ff8f33", markersize=5)
    ax_path.legend(loc="best")

    err = [
        math.sqrt(
            (data["tool_x"][i] - data["tool_ref_x"][i]) ** 2
            + (data["tool_y"][i] - data["tool_ref_y"][i]) ** 2
            + (data["tool_z"][i] - data["tool_ref_z"][i]) ** 2
        )
        for i in range(len(time_s))
    ]
    ax_err.set_xlabel("time (s)")
    ax_err.set_ylabel("tool error (m)")
    ax_err.set_xlim(time_s[0], time_s[-1])
    ax_err.set_ylim(0.0, max(err) * 1.08 if max(err) > 1.0e-6 else 1.0)
    ax_err.grid(True, alpha=0.25)
    ax_err.plot(time_s, err, color="#777777", linewidth=1.0, alpha=0.35)
    err_line, = ax_err.plot([], [], color="#202020", linewidth=1.8)
    time_marker = ax_err.axvline(time_s[0], color="#d43f3a", linewidth=1.2, alpha=0.85)
    status_text = ax_err.text(0.02, 0.94, "", transform=ax_err.transAxes, va="top")

    def update(frame: int):
        index = indices[frame]
        trail_start_frame = max(0, frame - max(1, args.trail))
        trail_start = indices[trail_start_frame]
        actual_line.set_data(actual_a[trail_start : index + 1], actual_b[trail_start : index + 1])
        target_line.set_data(ref_a[: index + 1], ref_b[: index + 1])
        actual_marker.set_data([actual_a[index]], [actual_b[index]])
        target_marker.set_data([ref_a[index]], [ref_b[index]])
        err_line.set_data(time_s[: index + 1], err[: index + 1])
        time_marker.set_xdata([time_s[index], time_s[index]])
        status_text.set_text(f"t={time_s[index]:.2f}s\nerr={err[index]:.3f}m")
        return actual_line, target_line, actual_marker, target_marker, err_line, time_marker, status_text

    anim = FuncAnimation(fig, update, frames=len(indices), interval=1000 / max(1, args.fps), blit=False)
    anim.save(out_path, writer=PillowWriter(fps=max(1, args.fps)))
    plt.close(fig)
    return out_path


def main() -> int:
    args = parse_args()
    try:
        out_path = render_gif(args)
    except Exception as exc:
        print(f"render_control_log_gif: {exc}")
        return 1
    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
