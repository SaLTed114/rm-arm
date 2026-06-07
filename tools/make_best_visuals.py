#!/usr/bin/env python3
"""Run representative benchmark cases and save MuJoCo-rendered GIFs under logs/best."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path


DEFAULT_SCENARIOS = (
    "joint_circle_j2j3_harsh",
    "joint_square_j2j3_harsh",
    "joint_insert_line_harsh",
    "teleop_wave_j2j3j5_harsh",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=Path("build"), help="CMake build directory")
    parser.add_argument("--config", type=Path, default=Path("configs/arm6_placeholder.yaml"), help="Analyzer config")
    parser.add_argument("--out-dir", type=Path, default=Path("logs/best"), help="Output directory")
    parser.add_argument("--scenario", action="append", help="Scenario to render; may be repeated")
    parser.add_argument("--ff", choices=("none", "gravity", "inverse"), default="gravity")
    parser.add_argument("--harsh", choices=("on", "off"), default="on")
    parser.add_argument("--contacts", choices=("on", "off"), default="on")
    parser.add_argument("--param-overrides", type=Path, default=None, help="Override params; default is latest tuning best.params")
    parser.add_argument("--baseline", action="store_true", help="Do not use param overrides")
    parser.add_argument("--fps", type=int, default=18, help="GIF and MuJoCo recording FPS")
    parser.add_argument("--keep-frames", action="store_true", help="Keep intermediate PPM frames")
    return parser.parse_args()


def root_dir() -> Path:
    return Path(__file__).resolve().parents[1]


def benchmark_exe(root: Path, build_dir: Path) -> Path:
    build = build_dir if build_dir.is_absolute() else root / build_dir
    exe = build / "sim_mujoco" / "armsim_control_benchmark.exe"
    if exe.exists():
        return exe
    return build / "sim_mujoco" / "armsim_control_benchmark"


def latest_best_params(root: Path) -> Path | None:
    runs = sorted((root / "logs" / "tuning_runs").glob("*/best.params"))
    return runs[-1] if runs else None


def run(command: list[str], cwd: Path) -> None:
    print("+ " + " ".join(command), flush=True)
    subprocess.run(command, cwd=cwd, check=True)


def compose_gif(frame_dir: Path, gif_path: Path, fps: int) -> int:
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError(
            "Pillow is required. Update the conda env with: conda env update -f environment.yml --prune"
        ) from exc

    frames = sorted(frame_dir.glob("frame_*.ppm"))
    if not frames:
        raise RuntimeError(f"no recorded frames found in {frame_dir}")

    images = [Image.open(frame).convert("P", palette=Image.ADAPTIVE, colors=128) for frame in frames]
    duration_ms = int(round(1000.0 / max(1, fps)))
    gif_path.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(
        gif_path,
        save_all=True,
        append_images=images[1:],
        duration=duration_ms,
        loop=0,
        optimize=True,
    )
    for image in images:
        image.close()
    return len(frames)


def main() -> int:
    args = parse_args()
    root = root_dir()
    exe = benchmark_exe(root, args.build_dir)
    if not exe.exists():
        print(f"Benchmark executable not found: {exe}", file=sys.stderr)
        return 1

    out_dir = args.out_dir if args.out_dir.is_absolute() else root / args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    param_overrides: Path | None = None
    if not args.baseline:
        param_overrides = args.param_overrides
        if param_overrides is None:
            param_overrides = latest_best_params(root)
        if param_overrides is not None and not param_overrides.is_absolute():
            param_overrides = root / param_overrides

    scenarios = tuple(args.scenario) if args.scenario else DEFAULT_SCENARIOS
    config_path = args.config if args.config.is_absolute() else root / args.config
    analyzer = root / "tools" / "analyze_control_log.py"
    manifest: dict[str, object] = {
        "param_overrides": str(param_overrides) if param_overrides else None,
        "ff": args.ff,
        "harsh": args.harsh,
        "contacts": args.contacts,
        "fps": args.fps,
        "cases": [],
    }

    if param_overrides and param_overrides.exists():
        shutil.copy2(param_overrides, out_dir / "best.params")

    for scenario in scenarios:
        case_dir = out_dir / scenario
        frame_dir = case_dir / "frames"
        if frame_dir.exists():
            shutil.rmtree(frame_dir)
        frame_dir.mkdir(parents=True, exist_ok=True)
        csv_path = case_dir / f"{scenario}.csv"
        gif_path = case_dir / f"{scenario}.gif"
        metrics_path = case_dir / f"{scenario}_metrics.json"

        command = [
            str(exe),
            scenario,
            str(csv_path),
            f"--ff={args.ff}",
            f"--harsh={args.harsh}",
            f"--contacts={args.contacts}",
            "--gui=off",
            f"--record-frames={frame_dir}",
            f"--record-fps={args.fps}",
        ]
        if param_overrides:
            command.append(f"--param-overrides={param_overrides}")
        run(command, root)

        run(
            [
                sys.executable,
                str(analyzer),
                str(csv_path),
                "--config",
                str(config_path),
                "--json-out",
                str(metrics_path),
            ],
            root,
        )
        frame_count = compose_gif(frame_dir, gif_path, args.fps)
        if not args.keep_frames:
            shutil.rmtree(frame_dir)

        with metrics_path.open("r", encoding="utf-8") as file:
            metrics = json.load(file)
        summary = metrics.get("summary", {})
        manifest["cases"].append(
            {
                "scenario": scenario,
                "csv": str(csv_path),
                "gif": str(gif_path),
                "metrics": str(metrics_path),
                "frames": frame_count,
                "q_err_max_abs": summary.get("q_err_max_abs"),
                "tool_pos_err_max_m": summary.get("tool_pos_err_max_m"),
                "tau_slew_rms_mean": summary.get("tau_slew_rms_mean"),
            }
        )
        print(f"Wrote {gif_path}")

    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"Wrote {out_dir / 'manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
