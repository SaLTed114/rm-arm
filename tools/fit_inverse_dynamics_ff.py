#!/usr/bin/env python3
"""Fit or check generated joint inverse-dynamics feedforward coefficients.

The generated C parameters are intentionally committed, so normal CMake builds
do not need Python or MuJoCo. MuJoCo sampling is done by the C-side
`armsim_sample_inverse_dynamics` tool; this script only reads its CSV, fits a
small linear feature model, and writes portable C parameters.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "configs" / "arm6_placeholder.yaml"
DEFAULT_OUTPUT = ROOT / "sim_mujoco" / "include" / "armsim" / "arm6_id_fit_params.h"


FEATURE_BIAS = 0
FEATURE_SIN_Q = 1
FEATURE_COS_Q = 2
FEATURE_DQ = 3
FEATURE_DQ_PRODUCT = 4
FEATURE_DDQ = 5
FEATURE_DDQ_SIN_Q = 6
FEATURE_DDQ_COS_Q = 7

FEATURE_NAMES = {
    FEATURE_BIAS: "JOINT_ID_FIT_FEATURE_BIAS",
    FEATURE_SIN_Q: "JOINT_ID_FIT_FEATURE_SIN_Q",
    FEATURE_COS_Q: "JOINT_ID_FIT_FEATURE_COS_Q",
    FEATURE_DQ: "JOINT_ID_FIT_FEATURE_DQ",
    FEATURE_DQ_PRODUCT: "JOINT_ID_FIT_FEATURE_DQ_PRODUCT",
    FEATURE_DDQ: "JOINT_ID_FIT_FEATURE_DDQ",
    FEATURE_DDQ_SIN_Q: "JOINT_ID_FIT_FEATURE_DDQ_SIN_Q",
    FEATURE_DDQ_COS_Q: "JOINT_ID_FIT_FEATURE_DDQ_COS_Q",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--samples-csv", type=Path, help="CSV from armsim_sample_inverse_dynamics")
    parser.add_argument("--fit", action="store_true", help="Fit from --samples-csv instead of emitting zero placeholder")
    parser.add_argument("--check", action="store_true", help="Fail if output differs from generated content")
    return parser.parse_args()


def load_config(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        data = json.loads(text)
    else:
        try:
            import yaml  # type: ignore
        except ImportError as exc:
            raise RuntimeError("PyYAML is required for YAML configs") from exc
        data = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise RuntimeError(f"{path} did not contain a YAML/JSON object")
    return data


def build_features(dof: int) -> list[tuple[int, int, int]]:
    features: list[tuple[int, int, int]] = [(FEATURE_BIAS, 0, 0)]
    for joint in range(dof):
        features.append((FEATURE_SIN_Q, joint, 0))
        features.append((FEATURE_COS_Q, joint, 0))
        features.append((FEATURE_DQ, joint, 0))
        features.append((FEATURE_DDQ, joint, 0))
    for a in range(dof):
        for b in range(a, dof):
            features.append((FEATURE_DQ_PRODUCT, a, b))
    for ddq_joint in range(dof):
        for q_joint in range(dof):
            features.append((FEATURE_DDQ_SIN_Q, ddq_joint, q_joint))
            features.append((FEATURE_DDQ_COS_Q, ddq_joint, q_joint))
    if len(features) > 128:
        raise RuntimeError(f"feature count {len(features)} exceeds JOINT_ID_FIT_FF_FEATURE_MAX")
    return features


def feature_values(features: list[tuple[int, int, int]], q: list[float], dq: list[float], ddq: list[float]) -> list[float]:
    values: list[float] = []
    for kind, a, b in features:
        if kind == FEATURE_BIAS:
            values.append(1.0)
        elif kind == FEATURE_SIN_Q:
            values.append(math.sin(q[a]))
        elif kind == FEATURE_COS_Q:
            values.append(math.cos(q[a]))
        elif kind == FEATURE_DQ:
            values.append(dq[a])
        elif kind == FEATURE_DQ_PRODUCT:
            values.append(dq[a] * dq[b])
        elif kind == FEATURE_DDQ:
            values.append(ddq[a])
        elif kind == FEATURE_DDQ_SIN_Q:
            values.append(ddq[a] * math.sin(q[b]))
        elif kind == FEATURE_DDQ_COS_Q:
            values.append(ddq[a] * math.cos(q[b]))
        else:
            values.append(0.0)
    return values


def solve_ridge(xtx: list[list[float]], xty: list[float], ridge: float = 1.0e-6) -> list[float]:
    n = len(xty)
    a = [row[:] for row in xtx]
    b = xty[:]
    for i in range(n):
        a[i][i] += ridge
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(a[row][col]))
        if abs(a[pivot][col]) < 1.0e-12:
            continue
        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
            b[col], b[pivot] = b[pivot], b[col]
        diag = a[col][col]
        for row in range(col + 1, n):
            factor = a[row][col] / diag
            a[row][col] = 0.0
            for k in range(col + 1, n):
                a[row][k] -= factor * a[col][k]
            b[row] -= factor * b[col]
    x = [0.0] * n
    for row in range(n - 1, -1, -1):
        value = b[row] - sum(a[row][col] * x[col] for col in range(row + 1, n))
        x[row] = value / a[row][row] if abs(a[row][row]) > 1.0e-12 else 0.0
    return x


def zero_fit(dof: int) -> tuple[list[tuple[int, int, int]], list[list[float]], dict[str, Any]]:
    return [(FEATURE_BIAS, 0, 0)], [[0.0] for _ in range(dof)], {"mode": "zero-placeholder"}


def load_sample_csv(path: Path, dof: int) -> tuple[list[list[float]], list[list[float]], list[list[float]], list[list[float]]]:
    q_rows: list[list[float]] = []
    dq_rows: list[list[float]] = []
    ddq_rows: list[list[float]] = []
    tau_rows: list[list[float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        if not reader.fieldnames:
            raise RuntimeError(f"{path} has no CSV header")
        for row in reader:
            q_rows.append([float(row[f"q{i + 1}"]) for i in range(dof)])
            dq_rows.append([float(row[f"dq{i + 1}"]) for i in range(dof)])
            ddq_rows.append([float(row[f"ddq{i + 1}"]) for i in range(dof)])
            tau_rows.append([float(row[f"tau{i + 1}"]) for i in range(dof)])
    if not q_rows:
        raise RuntimeError(f"{path} has no data rows")
    return q_rows, dq_rows, ddq_rows, tau_rows


def fit_from_csv(args: argparse.Namespace, config: dict[str, Any]) -> tuple[list[tuple[int, int, int]], list[list[float]], dict[str, Any]]:
    if not args.samples_csv:
        raise RuntimeError("--fit requires --samples-csv from armsim_sample_inverse_dynamics")

    dof = int(config["dof"])
    features = build_features(dof)
    q_rows, dq_rows, ddq_rows, tau_rows = load_sample_csv(args.samples_csv, dof)
    values_rows = [feature_values(features, q, dq, ddq) for q, dq, ddq in zip(q_rows, dq_rows, ddq_rows)]

    feature_count = len(features)
    xtx = [[0.0 for _ in range(feature_count)] for _ in range(feature_count)]
    for values in values_rows:
        for row in range(feature_count):
            for col in range(feature_count):
                xtx[row][col] += values[row] * values[col]

    coeffs: list[list[float]] = []
    for joint in range(dof):
        xty = [0.0 for _ in range(feature_count)]
        for values, tau in zip(values_rows, tau_rows):
            for feature_index in range(feature_count):
                xty[feature_index] += values[feature_index] * tau[joint]
        coeffs.append(solve_ridge(xtx, xty))

    err_rows: list[list[float]] = []
    for values, tau in zip(values_rows, tau_rows):
        err_rows.append([
            sum(coeffs[joint][feature] * values[feature] for feature in range(feature_count)) - tau[joint]
            for joint in range(dof)
        ])

    metrics = {
        "mode": "csv-fit",
        "samples": len(values_rows),
        "feature_count": len(features),
        "rms_error_nm": [
            math.sqrt(sum(row[joint] * row[joint] for row in err_rows) / len(err_rows))
            for joint in range(dof)
        ],
        "max_error_nm": [
            max(abs(row[joint]) for row in err_rows)
            for joint in range(dof)
        ],
    }
    return features, coeffs, metrics


def c_real(value: float) -> str:
    if abs(value) < 5.0e-12:
        return "ARM_REAL_ZERO"
    return f"ARM_REAL({value:.12g})"


def generate_header(config_path: Path, dof: int, features: list[tuple[int, int, int]], coeffs: list[list[float]], metrics: dict[str, Any]) -> str:
    lines = [
        f"/* Generated from {config_path.as_posix()} by tools/fit_inverse_dynamics_ff.py. Do not edit by hand. */",
        "#ifndef ARMSIM_ARM6_ID_FIT_PARAMS_H_",
        "#define ARMSIM_ARM6_ID_FIT_PARAMS_H_",
        "",
        '#include "arm_core/joint_id_fit_ff.h"',
        "",
        f"/* fit_mode={metrics.get('mode', 'unknown')} feature_count={len(features)} */",
        "#define ARMSIM_ARM6_ID_FIT_PARAMS \\",
        "  { \\",
        f"    {dof}u, \\",
        f"    {len(features)}u, \\",
        "    { \\",
    ]
    for kind, a, b in features:
        lines.append(f"      {{ {FEATURE_NAMES[kind]}, {a}u, {b}u }}, \\")
    lines.extend([
        "    }, \\",
        "    { \\",
    ])
    for joint in range(dof):
        row = ", ".join(c_real(value) for value in coeffs[joint])
        lines.append(f"      {{ {row} }}, \\")
    lines.extend([
        "    }, \\",
        "  }",
        "",
        "#endif",
    ])
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    try:
        config = load_config(args.config)
        dof = int(config["dof"])
        if args.fit:
            features, coeffs, metrics = fit_from_csv(args, config)
        else:
            features, coeffs, metrics = zero_fit(dof)
        content = generate_header(args.config, dof, features, coeffs, metrics)
        if args.check:
            existing = args.output.read_text(encoding="utf-8") if args.output.exists() else ""
            if existing != content:
                print(f"{args.output} is stale; rerun tools/fit_inverse_dynamics_ff.py", file=sys.stderr)
                return 1
            print(f"{args.output} is up to date ({metrics.get('mode')}).")
            return 0
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content, encoding="utf-8")
        print(json.dumps(metrics, indent=2))
    except Exception as exc:
        print(f"fit_inverse_dynamics_ff: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
