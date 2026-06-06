#!/usr/bin/env python3
"""Analyze source-agnostic joint-space control logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
from pathlib import Path
from typing import Any


PREFIX_ALIASES = {
    "q": ("q_filt", "q"),
    "dq": ("dq_filt", "dq"),
    "q_ref": ("q_ref",),
    "dq_ref": ("dq_ref",),
    "ddq_ref": ("ddq_ref",),
    "tau_cmd": ("tau_cmd", "cmd_tau_ff"),
    "q_meas": ("q_meas",),
    "dq_meas": ("dq_meas",),
    "tau_ff_gravity": ("tau_ff_gravity",),
    "tau_ff_model": ("tau_ff_model",),
    "tau_fb": ("tau_fb",),
    "mj_ctrl": ("mj_ctrl",),
    "tau_est": ("tau_est",),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="CSV control log to analyze")
    parser.add_argument("--config", type=Path, help="YAML or JSON arm config with joint torque limits")
    parser.add_argument("--json-out", type=Path, help="Write machine-readable metrics JSON")
    parser.add_argument("--settle-q-band", type=float, default=0.03, help="Settling position error band in rad")
    parser.add_argument("--settle-dq-band", type=float, default=0.08, help="Settling velocity error band in rad/s")
    parser.add_argument("--steady-window", type=float, default=0.2, help="Fraction of log used for steady-state chatter")
    return parser.parse_args()


def load_config(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() in (".json",):
        return json.loads(text)
    try:
        import yaml  # type: ignore
    except ImportError as exc:
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            raise RuntimeError("PyYAML is required for YAML configs; use JSON or install PyYAML") from exc
    else:
        data = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise RuntimeError(f"Config {path} did not contain an object")
    return data


def torque_limits_from_config(config: dict[str, Any]) -> list[float]:
    limits: list[float] = []
    joints = config.get("joints", [])
    if isinstance(joints, list):
        for joint in joints:
            if isinstance(joint, dict) and "torque_limit_nm" in joint:
                limits.append(float(joint["torque_limit_nm"]))
    return limits


def load_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        if not reader.fieldnames:
            raise RuntimeError(f"{path} has no CSV header")
        rows = list(reader)
    if not rows:
        raise RuntimeError(f"{path} has no data rows")
    return reader.fieldnames, rows


def prefixed_joint_columns(fields: list[str], prefixes: tuple[str, ...]) -> dict[int, str]:
    result: dict[int, str] = {}
    for prefix in prefixes:
        pattern = re.compile(rf"^{re.escape(prefix)}(\d+)$")
        for field in fields:
            match = pattern.match(field)
            if match:
                result[int(match.group(1))] = field
        if result:
            return result
    return result


def detect_columns(fields: list[str]) -> dict[str, dict[int, str]]:
    return {name: prefixed_joint_columns(fields, aliases) for name, aliases in PREFIX_ALIASES.items()}


def detect_dof(columns: dict[str, dict[int, str]]) -> int:
    required = ("q", "dq", "q_ref", "dq_ref", "tau_cmd")
    joint_sets = [set(columns[name].keys()) for name in required]
    common = set.intersection(*joint_sets)
    if not common:
        missing = [name for name in required if not columns[name]]
        raise RuntimeError(f"Missing required columns: {', '.join(missing)}")
    dof = max(common)
    expected = set(range(1, dof + 1))
    for name in required:
        if set(columns[name].keys()) & expected != expected:
            raise RuntimeError(f"Required column group '{name}' is incomplete for detected dof={dof}")
    return dof


def to_float(row: dict[str, str], column: str, default: float = 0.0) -> float:
    value = row.get(column, "")
    if value == "":
        return default
    return float(value)


def series(rows: list[dict[str, str]], column: str) -> list[float]:
    return [to_float(row, column) for row in rows]


def rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def max_abs(values: list[float]) -> float:
    if not values:
        return 0.0
    return max(abs(value) for value in values)


def mean_dt(times: list[float]) -> float:
    if len(times) < 2:
        return 0.0
    return statistics.fmean(max(0.0, b - a) for a, b in zip(times, times[1:]))


def slew(values: list[float], times: list[float]) -> list[float]:
    out: list[float] = []
    for prev_v, next_v, prev_t, next_t in zip(values, values[1:], times, times[1:]):
        dt = next_t - prev_t
        if dt > 0.0:
            out.append((next_v - prev_v) / dt)
    return out


def settling_time(
    times: list[float],
    q_err: list[float],
    dq_err: list[float],
    q_band: float,
    dq_band: float,
) -> float | None:
    for index, time_s in enumerate(times):
        q_ok = all(abs(value) <= q_band for value in q_err[index:])
        dq_ok = all(abs(value) <= dq_band for value in dq_err[index:])
        if q_ok and dq_ok:
            return time_s
    return None


def overshoot(q: list[float], q_ref: list[float]) -> float:
    if not q or not q_ref:
        return 0.0
    start = q_ref[0]
    target = q_ref[-1]
    delta = target - start
    if abs(delta) < 1e-6:
        return 0.0
    sign = 1.0 if delta > 0.0 else -1.0
    progress = [(value - start) * sign for value in q]
    return max(0.0, max(progress) - abs(delta))


def optional_series(columns: dict[str, dict[int, str]], name: str, joint: int, rows: list[dict[str, str]]) -> list[float] | None:
    column = columns[name].get(joint)
    if not column:
        return None
    return series(rows, column)


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    fields, rows = load_csv(args.log)
    columns = detect_columns(fields)
    dof = detect_dof(columns)
    times = series(rows, "time_s")
    config = load_config(args.config)
    torque_limits = torque_limits_from_config(config)

    steady_start = max(0, int(len(rows) * (1.0 - max(0.0, min(1.0, args.steady_window)))))
    joints: list[dict[str, Any]] = []
    active_joints: list[int] = []

    for joint in range(1, dof + 1):
        q = series(rows, columns["q"][joint])
        dq = series(rows, columns["dq"][joint])
        q_ref = series(rows, columns["q_ref"][joint])
        dq_ref = series(rows, columns["dq_ref"][joint])
        ddq_ref = optional_series(columns, "ddq_ref", joint, rows)
        tau_cmd = series(rows, columns["tau_cmd"][joint])
        q_err = [actual - ref for actual, ref in zip(q, q_ref)]
        dq_err = [actual - ref for actual, ref in zip(dq, dq_ref)]
        tau_slew = slew(tau_cmd, times)
        limit = torque_limits[joint - 1] if joint - 1 < len(torque_limits) else None
        if limit and limit > 0.0:
            saturation_ratio = sum(1 for value in tau_cmd if abs(value) >= 0.98 * limit) / len(tau_cmd)
        else:
            saturation_ratio = None

        mj_ctrl = optional_series(columns, "mj_ctrl", joint, rows)
        tau_fb = optional_series(columns, "tau_fb", joint, rows)
        tau_ff_gravity = optional_series(columns, "tau_ff_gravity", joint, rows)
        tau_ff_model = optional_series(columns, "tau_ff_model", joint, rows)
        q_meas = optional_series(columns, "q_meas", joint, rows)
        dq_meas = optional_series(columns, "dq_meas", joint, rows)
        ref_range = max(q_ref) - min(q_ref)
        if abs(ref_range) > 0.05:
            active_joints.append(joint)

        metrics: dict[str, Any] = {
            "joint": joint,
            "q_err_rms": rms(q_err),
            "q_err_max_abs": max_abs(q_err),
            "q_err_final_abs": abs(q_err[-1]) if q_err else 0.0,
            "dq_err_rms": rms(dq_err),
            "dq_err_max_abs": max_abs(dq_err),
            "overshoot_rad": overshoot(q, q_ref),
            "settling_time_s": settling_time(times, q_err, dq_err, args.settle_q_band, args.settle_dq_band),
            "steady_q_err_rms": rms(q_err[steady_start:]),
            "steady_dq_rms": rms(dq[steady_start:]),
            "steady_tau_cmd_slew_rms": rms(slew(tau_cmd[steady_start:], times[steady_start:])),
            "tau_cmd_rms": rms(tau_cmd),
            "tau_cmd_max_abs": max_abs(tau_cmd),
            "tau_slew_rms": rms(tau_slew),
            "saturation_ratio": saturation_ratio,
        }
        if mj_ctrl is not None:
            lag = [cmd - ctrl for cmd, ctrl in zip(tau_cmd, mj_ctrl)]
            metrics["actuator_lag_rms"] = rms(lag)
            metrics["actuator_lag_max_abs"] = max_abs(lag)
        if tau_fb is not None:
            metrics["tau_fb_rms"] = rms(tau_fb)
            metrics["steady_tau_fb_rms"] = rms(tau_fb[steady_start:])
        if tau_ff_gravity is not None:
            metrics["tau_ff_gravity_rms"] = rms(tau_ff_gravity)
        if tau_ff_model is not None:
            metrics["tau_ff_model_rms"] = rms(tau_ff_model)
        if ddq_ref is not None:
            metrics["ddq_ref_rms"] = rms(ddq_ref)
            metrics["ddq_ref_max_abs"] = max_abs(ddq_ref)
        if q_meas is not None:
            metrics["q_filter_delta_rms"] = rms([filt - meas for filt, meas in zip(q, q_meas)])
        if dq_meas is not None:
            metrics["dq_filter_delta_rms"] = rms([filt - meas for filt, meas in zip(dq, dq_meas)])
        joints.append(metrics)

    passive_joints = [joint for joint in range(1, dof + 1) if joint not in active_joints]
    if active_joints:
        passive_q_max = max((joints[joint - 1]["q_err_max_abs"] for joint in passive_joints), default=0.0)
        passive_dq_max = max((joints[joint - 1]["dq_err_max_abs"] for joint in passive_joints), default=0.0)
    else:
        passive_q_max = None
        passive_dq_max = None

    summary = {
        "log": str(args.log),
        "samples": len(rows),
        "dof": dof,
        "duration_s": times[-1] - times[0] if len(times) >= 2 else 0.0,
        "dt_mean_s": mean_dt(times),
        "q_err_rms_mean": statistics.fmean(item["q_err_rms"] for item in joints),
        "q_err_max_abs": max(item["q_err_max_abs"] for item in joints),
        "dq_err_rms_mean": statistics.fmean(item["dq_err_rms"] for item in joints),
        "dq_err_max_abs": max(item["dq_err_max_abs"] for item in joints),
        "steady_dq_rms_mean": statistics.fmean(item["steady_dq_rms"] for item in joints),
        "tau_slew_rms_mean": statistics.fmean(item["tau_slew_rms"] for item in joints),
        "active_joints": active_joints,
        "cross_coupling": {
            "passive_q_err_max_abs": passive_q_max,
            "passive_dq_err_max_abs": passive_dq_max,
        },
    }
    saturation_values = [
        (item["joint"], item["saturation_ratio"])
        for item in joints
        if item["saturation_ratio"] is not None
    ]
    if saturation_values:
        summary["saturation_ratio_max"] = max(value for _, value in saturation_values)
        summary["saturation_worst_joint"] = max(saturation_values, key=lambda item: item[1])[0]

    return {"summary": summary, "joints": joints}


def print_summary(result: dict[str, Any]) -> None:
    summary = result["summary"]
    print(f"log: {summary['log']}")
    print(
        "samples={samples} dof={dof} duration={duration_s:.3f}s dt_mean={dt_mean_s:.6f}s".format(
            **summary
        )
    )
    print(
        "tracking: q_rms_mean={q_err_rms_mean:.5f} rad q_max={q_err_max_abs:.5f} rad "
        "dq_rms_mean={dq_err_rms_mean:.5f} rad/s dq_max={dq_err_max_abs:.5f} rad/s".format(
            **summary
        )
    )
    print(
        "chatter: steady_dq_rms_mean={steady_dq_rms_mean:.5f} rad/s "
        "tau_slew_rms_mean={tau_slew_rms_mean:.2f} Nm/s".format(**summary)
    )
    if "saturation_ratio_max" in summary:
        print(
            "saturation: max_ratio={:.3f} worst_joint=J{}".format(
                summary["saturation_ratio_max"], summary["saturation_worst_joint"]
            )
        )
    cc = summary["cross_coupling"]
    if cc["passive_q_err_max_abs"] is None:
        print("cross-coupling: n/a (no active reference motion detected)")
    else:
        print(
            "cross-coupling: passive_q_max={:.5f} rad passive_dq_max={:.5f} rad/s".format(
                cc["passive_q_err_max_abs"], cc["passive_dq_err_max_abs"]
            )
        )
    print("per-joint:")
    for item in result["joints"]:
        settle = item["settling_time_s"]
        settle_text = "n/a" if settle is None else f"{settle:.3f}s"
        saturation = item["saturation_ratio"]
        saturation_text = "n/a" if saturation is None else f"{saturation:.3f}"
        print(
            "  J{joint}: q_rms={q_err_rms:.5f} q_max={q_err_max_abs:.5f} "
            "overshoot={overshoot_rad:.5f} settle={settle} steady_dq={steady_dq_rms:.5f} "
            "sat={sat}".format(
                joint=item["joint"],
                q_err_rms=item["q_err_rms"],
                q_err_max_abs=item["q_err_max_abs"],
                overshoot_rad=item["overshoot_rad"],
                settle=settle_text,
                steady_dq_rms=item["steady_dq_rms"],
                sat=saturation_text,
            )
        )


def main() -> int:
    args = parse_args()
    try:
        result = analyze(args)
        print_summary(result)
        if args.json_out:
            args.json_out.parent.mkdir(parents=True, exist_ok=True)
            args.json_out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    except Exception as exc:
        print(f"analyze_control_log: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
