#!/usr/bin/env python3
"""Run ArmSim control benchmarks, analyze logs, and print a compact tuning report."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


DEFAULT_SCENARIOS = ("hold_zero_harsh", "step_j2_harsh", "coupled_j2j3_harsh", "step_j5_harsh")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=Path("build"), help="CMake build directory")
    parser.add_argument("--config", type=Path, default=Path("configs/arm6_placeholder.yaml"), help="Arm config")
    parser.add_argument("--out-dir", type=Path, default=Path("logs"), help="Output log directory")
    parser.add_argument("--scenario", action="append", choices=DEFAULT_SCENARIOS, help="Scenario to run")
    parser.add_argument("--skip-run", action="store_true", help="Analyze existing CSV logs without rerunning sim")
    parser.add_argument("--verbose", action="store_true", help="Print full analyzer output")
    return parser.parse_args()


def benchmark_exe(build_dir: Path) -> Path:
    exe = build_dir / "sim_mujoco" / "armsim_control_benchmark.exe"
    if exe.exists():
        return exe
    exe = build_dir / "sim_mujoco" / "armsim_control_benchmark"
    return exe


def run_command(command: list[str], cwd: Path, capture: bool = False) -> str:
    print("+ " + " ".join(command), flush=True)
    if capture:
        completed = subprocess.run(command, cwd=cwd, check=True, text=True, capture_output=True)
        return completed.stdout
    subprocess.run(command, cwd=cwd, check=True)
    return ""


def run_scenario(root: Path, exe: Path, scenario: str, csv_path: Path, skip_run: bool) -> None:
    if skip_run:
        if not csv_path.exists():
            raise RuntimeError(f"{csv_path} does not exist; remove --skip-run or generate it first")
        return
    run_command([str(exe), scenario, str(csv_path)], root)


def analyze_scenario(
    root: Path,
    scenario: str,
    csv_path: Path,
    config_path: Path,
    json_path: Path,
    verbose: bool,
) -> dict[str, Any]:
    analyzer = root / "tools" / "analyze_control_log.py"
    output = run_command(
        [
            sys.executable,
            str(analyzer),
            str(csv_path),
            "--config",
            str(config_path),
            "--json-out",
            str(json_path),
        ],
        root,
        capture=not verbose,
    )
    if verbose and output:
        print(output)
    return json.loads(json_path.read_text(encoding="utf-8"))


def worst_joint(result: dict[str, Any], metric: str) -> dict[str, Any]:
    return max(result["joints"], key=lambda item: item.get(metric) or 0.0)


def format_optional(value: Any, precision: int = 3) -> str:
    if value is None:
        return "n/a"
    return f"{float(value):.{precision}f}"


def print_report(results: dict[str, dict[str, Any]]) -> None:
    print("\nBenchmark summary")
    print("scenario                  q_max   dq_max  sat_max  tau_slew  worst_q  worst_sat")
    for scenario, result in results.items():
        summary = result["summary"]
        q_worst = worst_joint(result, "q_err_max_abs")
        sat_worst = worst_joint(result, "saturation_ratio")
        print(
            f"{scenario:24s} "
            f"{summary['q_err_max_abs']:6.3f} "
            f"{summary['dq_err_max_abs']:7.3f} "
            f"{format_optional(summary.get('saturation_ratio_max'), 3):>7s} "
            f"{summary['tau_slew_rms_mean']:9.1f} "
            f"J{q_worst['joint']:<7d} "
            f"J{sat_worst['joint']}"
        )

    print("\nQuick notes")
    for scenario, result in results.items():
        summary = result["summary"]
        q_worst = worst_joint(result, "q_err_max_abs")
        slew_worst = worst_joint(result, "tau_slew_rms")
        sat = summary.get("saturation_ratio_max")
        notes: list[str] = []
        if q_worst["q_err_max_abs"] > 0.25:
            notes.append(f"J{q_worst['joint']} has large transient error")
        if q_worst["overshoot_rad"] > 0.06:
            notes.append(f"J{q_worst['joint']} overshoot is still visible")
        if sat is not None and sat > 0.05:
            notes.append("torque saturation is becoming part of the response")
        if slew_worst["tau_slew_rms"] > 250.0:
            notes.append(f"J{slew_worst['joint']} torque slew is high")
        cc = summary.get("cross_coupling", {})
        passive_q = cc.get("passive_q_err_max_abs")
        if passive_q is not None and passive_q > 0.05:
            notes.append("passive-joint cross coupling is noticeable")
        if not notes:
            notes.append("no obvious red flag from coarse thresholds")
        print(f"- {scenario}: " + "; ".join(notes))


def main() -> int:
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    build_dir = args.build_dir if args.build_dir.is_absolute() else root / args.build_dir
    config_path = args.config if args.config.is_absolute() else root / args.config
    out_dir = args.out_dir if args.out_dir.is_absolute() else root / args.out_dir
    scenarios = tuple(args.scenario) if args.scenario else DEFAULT_SCENARIOS
    exe = benchmark_exe(build_dir)
    if not args.skip_run and not exe.exists():
        print(f"Benchmark executable not found: {exe}", file=sys.stderr)
        return 1

    out_dir.mkdir(parents=True, exist_ok=True)
    results: dict[str, dict[str, Any]] = {}
    try:
        for scenario in scenarios:
            csv_path = out_dir / f"control_benchmark_{scenario}.csv"
            json_path = out_dir / f"control_benchmark_{scenario}_metrics.json"
            run_scenario(root, exe, scenario, csv_path, args.skip_run)
            results[scenario] = analyze_scenario(root, scenario, csv_path, config_path, json_path, args.verbose)
    except subprocess.CalledProcessError as exc:
        print(f"Command failed with exit code {exc.returncode}", file=sys.stderr)
        return exc.returncode
    except Exception as exc:
        print(f"run_control_benchmarks: {exc}", file=sys.stderr)
        return 1

    print_report(results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
