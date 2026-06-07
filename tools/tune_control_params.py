#!/usr/bin/env python3
"""Model-informed CMA-style tuning for ArmSim control parameters.

The tuner does not edit the canonical YAML and does not rebuild. It writes full
candidate YAML snapshots plus small runtime override files consumed by
armsim_control_benchmark via --param-overrides.
"""

from __future__ import annotations

import argparse
import copy
import datetime as _dt
import json
import math
import random
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

try:
    from tqdm import tqdm  # type: ignore
except ModuleNotFoundError:
    tqdm = None

try:
    import yaml  # type: ignore
except ModuleNotFoundError as exc:
    raise SystemExit("PyYAML is required. Install it with: conda env create -f environment.yml") from exc

DEFAULT_MAIN_SCENARIOS = (
    "hold_zero_harsh",
    "step_j2_harsh",
    "step_j3_harsh",
    "coupled_j2j3_harsh",
    "sine_j2_harsh",
    "joint_circle_j2j3_harsh",
    "joint_square_j2j3_harsh",
    "joint_insert_line_harsh",
)
DEFAULT_CHECK_SCENARIOS = (
    "tool_circle_xz_harsh",
    "tool_square_xz_harsh",
    "tool_insert_line_harsh",
    "tool_circle_xy_harsh",
    "tool_circle_yz_harsh",
    "tool_square_xy_harsh",
    "tool_square_yz_harsh",
    "straight_arm_lift_harsh",
)
SCENARIO_WEIGHTS = {
    "hold_zero_harsh": 0.8,
    "step_j2_harsh": 1.0,
    "step_j3_harsh": 1.0,
    "coupled_j2j3_harsh": 1.2,
    "sine_j2_harsh": 0.8,
    "joint_circle_j2j3_harsh": 1.6,
    "joint_square_j2j3_harsh": 1.4,
    "joint_insert_line_harsh": 1.8,
}
BIG_PENALTY = 1.0e9
VERBOSE_COMMANDS = False
HEAVY_JOINTS = (1, 2)  # zero-based J2/J3.
WRIST_JOINTS = (3, 4, 5)


@dataclass(frozen=True)
class DesignVariable:
    name: str
    lo: float
    hi: float


DESIGN_VARIABLES = (
    DesignVariable("heavy_wn_scale", 0.65, 1.35),
    DesignVariable("wrist_wn_scale", 0.75, 1.45),
    DesignVariable("heavy_zeta", 0.75, 1.55),
    DesignVariable("wrist_zeta", 0.70, 1.45),
    DesignVariable("heavy_speed_scale", 0.65, 1.35),
    DesignVariable("heavy_acc_scale", 0.55, 1.55),
    DesignVariable("heavy_jerk_scale", 0.45, 1.80),
    DesignVariable("wrist_speed_scale", 0.75, 1.55),
    DesignVariable("wrist_acc_scale", 0.75, 1.70),
    DesignVariable("wrist_jerk_scale", 0.70, 1.90),
    DesignVariable("filter_scale", 0.75, 1.80),
    DesignVariable("output_rate_scale", 0.35, 3.00),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--budget", type=int, default=30, help="Total candidates including baseline and model seed")
    parser.add_argument("--seed", type=int, default=1, help="Random seed")
    parser.add_argument("--profile", default="teleop_core", help="Tuning profile name")
    parser.add_argument("--optimizer", choices=("cma", "random"), default="cma", help="Candidate optimizer")
    parser.add_argument("--config", type=Path, default=Path("configs/arm6_placeholder.yaml"))
    parser.add_argument("--build-dir", type=Path, default=Path("build"), help="Existing CMake build directory")
    parser.add_argument("--out-root", type=Path, default=Path("logs/tuning_runs"))
    parser.add_argument("--scenario", action="append", help="Main-loss scenario; repeat to override defaults")
    parser.add_argument("--check-scenario", action="append", help="Check-only scenario; repeat to override defaults")
    parser.add_argument("--allow-dirty", action="store_true", help="Allow tracked worktree changes while tuning")
    parser.add_argument("--verbose-commands", action="store_true", help="Print every external command")
    parser.add_argument("--timeout-s", type=float, default=180.0, help="Timeout per benchmark/analyzer command")
    return parser.parse_args()


def root_dir() -> Path:
    return Path(__file__).resolve().parents[1]


def rel_or_abs(path: Path, root: Path) -> Path:
    return path if path.is_absolute() else root / path


def progress_iter(iterable: Iterable[Any], desc: str, unit: str = "it", leave: bool = True) -> Iterable[Any]:
    if tqdm is None:
        return iterable
    return tqdm(iterable, desc=desc, unit=unit, leave=leave)


def run_command(command: list[str], cwd: Path, timeout_s: float) -> subprocess.CompletedProcess[str]:
    if VERBOSE_COMMANDS:
        print("+ " + " ".join(command), flush=True)
    return subprocess.run(command, cwd=cwd, check=True, text=True, capture_output=True, timeout=timeout_s)


def git_tracked_dirty(root: Path) -> str:
    completed = run_command(["git", "status", "--porcelain", "--untracked-files=no"], root, timeout_s=30.0)
    return completed.stdout.strip()


def load_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"{path} must contain a top-level mapping")
    return data


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    text = yaml.safe_dump(data, sort_keys=False, allow_unicode=False, default_flow_style=False)
    path.write_text(text, encoding="utf-8", newline="\n")


def benchmark_exe(build_dir: Path) -> Path:
    exe = build_dir / "sim_mujoco" / "armsim_control_benchmark.exe"
    if exe.exists():
        return exe
    return build_dir / "sim_mujoco" / "armsim_control_benchmark"


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def dot(a: list[float], b: list[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def add(a: list[float], b: list[float]) -> list[float]:
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def sub(a: list[float], b: list[float]) -> list[float]:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def scale_vec(a: list[float], s: float) -> list[float]:
    return [a[0] * s, a[1] * s, a[2] * s]


def norm(a: list[float]) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: list[float]) -> list[float]:
    n = norm(a)
    if n <= 1.0e-12:
        return [1.0, 0.0, 0.0]
    return scale_vec(a, 1.0 / n)


def mat_vec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [
        dot(matrix[0], vector),
        dot(matrix[1], vector),
        dot(matrix[2], vector),
    ]


def mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    cols = [[b[0][i], b[1][i], b[2][i]] for i in range(3)]
    return [[dot(a[row], cols[col]) for col in range(3)] for row in range(3)]


def rotation_matrix(axis: list[float], angle: float) -> list[list[float]]:
    x, y, z = normalize(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return [
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ]


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    sorted_values = sorted(values)
    index = int(round(clamp(fraction, 0.0, 1.0) * (len(sorted_values) - 1)))
    return sorted_values[index]


def geom_mass_and_com(geom: dict[str, Any]) -> tuple[float, list[float]]:
    mass = float(geom.get("mass", 0.0))
    if mass <= 0.0:
        density = float(geom.get("density", 0.0))
        mass = 0.02 if density <= 0.0 else min(0.20, density * 1.0e-6)
    if "fromto" in geom:
        values = [float(v) for v in geom["fromto"]]
        com = [(values[0] + values[3]) * 0.5, (values[1] + values[4]) * 0.5, (values[2] + values[5]) * 0.5]
    else:
        com = [float(v) for v in geom.get("pos", [0.0, 0.0, 0.0])]
    return mass, com


def joint_index_by_name(config: dict[str, Any]) -> dict[str, int]:
    return {str(joint["name"]): index for index, joint in enumerate(config["joints"])}


def collect_body_dynamics(
    body: dict[str, Any],
    config: dict[str, Any],
    q: list[float],
    joint_lookup: dict[str, int],
    parent_pos: list[float],
    parent_rot: list[list[float]],
    active_joints: list[int],
    joint_positions: list[list[float]],
    joint_axes: list[list[float]],
    inertia_accum: list[float],
    gravity_tau_abs: list[list[float]],
) -> None:
    body_pos_local = [float(v) for v in body.get("pos", [0.0, 0.0, 0.0])]
    body_pos = add(parent_pos, mat_vec(parent_rot, body_pos_local))
    body_rot = parent_rot
    next_active = list(active_joints)

    joint_name = body.get("joint")
    if joint_name is not None:
        joint_index = joint_lookup[str(joint_name)]
        axis_local = [float(v) for v in config["joints"][joint_index]["axis"]]
        axis_world = normalize(mat_vec(parent_rot, axis_local))
        joint_positions[joint_index] = body_pos
        joint_axes[joint_index] = axis_world
        next_active.append(joint_index)
        body_rot = mat_mul(parent_rot, rotation_matrix(axis_local, q[joint_index]))

    for geom in body.get("geoms", []):
        mass, com_local = geom_mass_and_com(geom)
        com_world = add(body_pos, mat_vec(body_rot, com_local))
        gravity = [0.0, 0.0, -9.81 * mass]
        for joint_index in next_active:
            r = sub(com_world, joint_positions[joint_index])
            axis = joint_axes[joint_index]
            lever = norm(cross(axis, r))
            inertia_accum[joint_index] += mass * lever * lever
            gravity_tau_abs[joint_index].append(abs(dot(cross(r, gravity), axis)))

    for child in body.get("bodies", []):
        collect_body_dynamics(
            child,
            config,
            q,
            joint_lookup,
            body_pos,
            body_rot,
            next_active,
            joint_positions,
            joint_axes,
            inertia_accum,
            gravity_tau_abs,
        )


def sample_model_diagnostics(config: dict[str, Any], sample_count: int = 81) -> dict[str, Any]:
    dof = int(config["dof"])
    rng = random.Random(0xA651)
    inertia_samples = [[] for _ in range(dof)]
    gravity_samples = [[] for _ in range(dof)]
    joint_lookup = joint_index_by_name(config)
    world_bodies = config["mujoco"]["worldbody"].get("bodies", [])

    postures: list[list[float]] = [[0.0] * dof]
    for joint in HEAVY_JOINTS:
        q = [0.0] * dof
        q[joint] = -0.45
        postures.append(q)
        q = [0.0] * dof
        q[joint] = 0.45
        postures.append(q)
    while len(postures) < sample_count:
        q = []
        for joint in config["joints"]:
            lo = float(joint["q_min_rad"])
            hi = float(joint["q_max_rad"])
            span = min(0.65, 0.30 * (hi - lo))
            q.append(clamp(rng.uniform(-span, span), lo, hi))
        postures.append(q)

    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for q in postures:
        inertia_accum = [0.0] * dof
        gravity_tau_abs = [[] for _ in range(dof)]
        joint_positions = [[0.0, 0.0, 0.0] for _ in range(dof)]
        joint_axes = [[1.0, 0.0, 0.0] for _ in range(dof)]
        for body in world_bodies:
            collect_body_dynamics(
                body,
                config,
                q,
                joint_lookup,
                [0.0, 0.0, 0.0],
                identity,
                [],
                joint_positions,
                joint_axes,
                inertia_accum,
                gravity_tau_abs,
            )
        for i in range(dof):
            inertia_samples[i].append(max(0.002, inertia_accum[i]))
            gravity_samples[i].append(max(gravity_tau_abs[i]) if gravity_tau_abs[i] else 0.0)

    joints = []
    for i, joint in enumerate(config["joints"]):
        torque_limit = float(joint["torque_limit_nm"])
        inertia_p50 = percentile(inertia_samples[i], 0.50)
        inertia_p80 = percentile(inertia_samples[i], 0.80)
        gravity_p95 = percentile(gravity_samples[i], 0.95)
        margin = max(0.15 * torque_limit, torque_limit - gravity_p95)
        joints.append(
            {
                "joint": i + 1,
                "name": joint["name"],
                "torque_limit_nm": torque_limit,
                "effective_inertia_p50": inertia_p50,
                "effective_inertia_p80": inertia_p80,
                "gravity_tau_abs_p95": gravity_p95,
                "torque_margin_nm": margin,
            }
        )
    return {"sample_count": len(postures), "joints": joints}


def clamp_relative(value: float, reference: float, lo_scale: float, hi_scale: float) -> float:
    if reference <= 0.0:
        return value
    return clamp(value, reference * lo_scale, reference * hi_scale)


def build_model_seed_config(config: dict[str, Any], diagnostics: dict[str, Any]) -> dict[str, Any]:
    seeded = copy.deepcopy(config)
    control = seeded["control"]
    base_control = config["control"]
    for index, diag in enumerate(diagnostics["joints"]):
        torque_limit = float(diag["torque_limit_nm"])
        inertia = float(diag["effective_inertia_p80"])
        margin = float(diag["torque_margin_nm"])
        heavy = index in HEAVY_JOINTS
        wrist = index in WRIST_JOINTS
        wn = 5.0 if heavy else (10.0 if wrist else 4.5)
        zeta = 1.15 if heavy else (0.95 if wrist else 1.05)
        kp = inertia * wn * wn
        kd = 2.0 * zeta * inertia * wn
        out_limit = min(0.55 * torque_limit, max(0.20 * torque_limit, 0.70 * margin))

        base_pd = base_control["pd"][index]
        pd = control["pd"][index]
        pd["kp"] = clamp_relative(kp, float(base_pd["kp"]), 0.55, 1.65)
        pd["kd"] = clamp_relative(kd, float(base_pd["kd"]), 0.45, 1.80)
        pd["kd"] = min(float(pd["kd"]), 0.55 * float(pd["kp"]))
        pd["out_limit"] = clamp_relative(out_limit, float(base_pd["out_limit"]), 0.65, 1.40)

        base_filter = float(base_control["state_filter"][index]["dq_time_constant_s"])
        if heavy:
            control["state_filter"][index]["dq_time_constant_s"] = clamp(max(base_filter, 0.026), 0.012, 0.060)
        elif wrist:
            control["state_filter"][index]["dq_time_constant_s"] = clamp(base_filter, 0.006, 0.020)

    ref = control["reference"]
    base_ref = base_control["reference"]
    for index in range(int(config["dof"])):
        heavy = index in HEAVY_JOINTS
        wrist = index in WRIST_JOINTS
        speed_scale = 0.85 if heavy else (1.15 if wrist else 1.00)
        acc_scale = 0.85 if heavy else (1.10 if wrist else 1.00)
        jerk_scale = 0.75 if heavy else (1.15 if wrist else 1.00)
        ref["dq_limits_rad_s"][index] = float(base_ref["dq_limits_rad_s"][index]) * speed_scale
        ref["ddq_limits_rad_s2"][index] = float(base_ref["ddq_limits_rad_s2"][index]) * acc_scale
        ref["dddq_limits_rad_s3"][index] = float(base_ref["dddq_limits_rad_s3"][index]) * jerk_scale

    base_rate = float(base_control["output_limiter"]["tau_rate_limits_nm_s"][0])
    rate = clamp(base_rate * 0.25, 500.0, 1800.0)
    rates = control["output_limiter"]["tau_rate_limits_nm_s"]
    for i in range(len(rates)):
        rates[i] = rate

    seeded["tuning"] = {
        "stage": "model_seed",
        "note": "Generated from model diagnostics by tools/tune_control_params.py; not canonical config.",
    }
    return seeded


def params_snapshot(config: dict[str, Any]) -> dict[str, Any]:
    control = config["control"]
    joints = tuple(range(int(config["dof"])))
    return {
        "pd": [dict(control["pd"][joint]) for joint in joints],
        "state_filter": [dict(control["state_filter"][joint]) for joint in joints],
        "reference": {
            "dq_limits_rad_s": list(control["reference"]["dq_limits_rad_s"]),
            "ddq_limits_rad_s2": list(control["reference"]["ddq_limits_rad_s2"]),
            "dddq_limits_rad_s3": list(control["reference"]["dddq_limits_rad_s3"]),
        },
        "tau_rate_limit_nm_s": control["output_limiter"]["tau_rate_limits_nm_s"][0],
    }


def write_override_file(path: Path, config: dict[str, Any]) -> None:
    control = config["control"]
    lines = [
        "# Generated by tools/tune_control_params.py.",
        "# Joint numbers are one-based: pd.2.* means J2.",
    ]
    for joint in range(int(config["dof"])):
        j = joint + 1
        pd = control["pd"][joint]
        lines.append(f"pd.{j}.kp={float(pd['kp']):.12g}")
        lines.append(f"pd.{j}.kd={float(pd['kd']):.12g}")
        lines.append(f"pd.{j}.out_limit={float(pd['out_limit']):.12g}")
        filt = control["state_filter"][joint]
        lines.append(f"state_filter.{j}.dq_time_constant_s={float(filt['dq_time_constant_s']):.12g}")
        ref = control["reference"]
        lines.append(f"shaper.{j}.dq_limit_rad_s={float(ref['dq_limits_rad_s'][joint]):.12g}")
        lines.append(f"shaper.{j}.ddq_limit_rad_s2={float(ref['ddq_limits_rad_s2'][joint]):.12g}")
        lines.append(f"shaper.{j}.dddq_limit_rad_s3={float(ref['dddq_limits_rad_s3'][joint]):.12g}")
    lines.append(f"output_limiter.tau_rate_limit_nm_s={float(control['output_limiter']['tau_rate_limits_nm_s'][0]):.12g}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def set_group_pd_from_seed(config: dict[str, Any], seed_config: dict[str, Any], joints: tuple[int, ...], wn_scale: float, zeta: float) -> None:
    for joint in joints:
        seed_pd = seed_config["control"]["pd"][joint]
        kp = float(seed_pd["kp"]) * wn_scale * wn_scale
        kd = float(seed_pd["kd"]) * wn_scale * zeta
        config["control"]["pd"][joint]["kp"] = kp
        config["control"]["pd"][joint]["kd"] = min(kd, 0.55 * kp)
        config["control"]["pd"][joint]["out_limit"] = float(seed_pd["out_limit"])


def set_group_shaper_from_seed(
    config: dict[str, Any],
    seed_config: dict[str, Any],
    joints: tuple[int, ...],
    speed_scale: float,
    acc_scale: float,
    jerk_scale: float,
) -> None:
    ref = config["control"]["reference"]
    seed_ref = seed_config["control"]["reference"]
    for joint in joints:
        ref["dq_limits_rad_s"][joint] = float(seed_ref["dq_limits_rad_s"][joint]) * speed_scale
        ref["ddq_limits_rad_s2"][joint] = float(seed_ref["ddq_limits_rad_s2"][joint]) * acc_scale
        ref["dddq_limits_rad_s3"][joint] = float(seed_ref["dddq_limits_rad_s3"][joint]) * jerk_scale


def design_vector_to_scales(vector: list[float]) -> dict[str, float]:
    scales: dict[str, float] = {}
    for value, variable in zip(vector, DESIGN_VARIABLES):
        clipped = clamp(float(value), -2.5, 2.5)
        mid = math.sqrt(variable.lo * variable.hi)
        half_log_span = 0.5 * math.log(variable.hi / variable.lo)
        scales[variable.name] = clamp(mid * math.exp(half_log_span * clipped), variable.lo, variable.hi)
    return scales


def make_scaled_candidate(index: int, stage: str, seed_config: dict[str, Any], vector: list[float]) -> dict[str, Any]:
    config = copy.deepcopy(seed_config)
    scales = design_vector_to_scales(vector)
    set_group_pd_from_seed(config, seed_config, HEAVY_JOINTS, scales["heavy_wn_scale"], scales["heavy_zeta"])
    set_group_pd_from_seed(config, seed_config, WRIST_JOINTS, scales["wrist_wn_scale"], scales["wrist_zeta"])
    set_group_shaper_from_seed(
        config,
        seed_config,
        HEAVY_JOINTS,
        scales["heavy_speed_scale"],
        scales["heavy_acc_scale"],
        scales["heavy_jerk_scale"],
    )
    set_group_shaper_from_seed(
        config,
        seed_config,
        WRIST_JOINTS,
        scales["wrist_speed_scale"],
        scales["wrist_acc_scale"],
        scales["wrist_jerk_scale"],
    )

    for joint in range(int(config["dof"])):
        seed_tau = float(seed_config["control"]["state_filter"][joint]["dq_time_constant_s"])
        config["control"]["state_filter"][joint]["dq_time_constant_s"] = clamp(seed_tau * scales["filter_scale"], 0.006, 0.070)

    seed_rate = float(seed_config["control"]["output_limiter"]["tau_rate_limits_nm_s"][0])
    rate = clamp(seed_rate * scales["output_rate_scale"], 300.0, 3200.0)
    rates = config["control"]["output_limiter"]["tau_rate_limits_nm_s"]
    for i in range(len(rates)):
        rates[i] = rate

    config["tuning"] = {
        "candidate": index,
        "stage": stage,
        "design_scales": scales,
        "note": "Generated by tools/tune_control_params.py; not canonical config.",
    }
    return config


def run_one_scenario(
    root: Path,
    exe: Path,
    scenario: str,
    config_for_analyzer: Path,
    override_path: Path,
    out_dir: Path,
    timeout_s: float,
) -> dict[str, Any]:
    csv_path = out_dir / f"{scenario}.csv"
    json_path = out_dir / f"{scenario}_metrics.json"
    run_command(
        [
            str(exe),
            scenario,
            str(csv_path),
            "--ff=gravity",
            "--harsh=on",
            "--contacts=on",
            "--gui=off",
            f"--param-overrides={override_path}",
        ],
        root,
        timeout_s=timeout_s,
    )
    run_command(
        [
            sys.executable,
            str(root / "tools" / "analyze_control_log.py"),
            str(csv_path),
            "--config",
            str(config_for_analyzer),
            "--json-out",
            str(json_path),
        ],
        root,
        timeout_s=timeout_s,
    )
    metrics = json.loads(json_path.read_text(encoding="utf-8"))
    return {"csv": str(csv_path), "json": str(json_path), "summary": metrics.get("summary", {})}


def checked_metric(summary: dict[str, Any], key: str, reason: list[str]) -> float:
    value = summary.get(key)
    if value is None:
        reason.append(f"missing {key}")
        return math.nan
    value = float(value)
    if not math.isfinite(value):
        reason.append(f"non-finite {key}")
        return math.nan
    return value


def scenario_loss(scenario: str, summary: dict[str, Any]) -> tuple[float, list[str]]:
    reasons: list[str] = []
    q_rms = checked_metric(summary, "q_err_rms_mean", reasons)
    q_max = checked_metric(summary, "q_err_max_abs", reasons)
    dq_rms = checked_metric(summary, "dq_err_rms_mean", reasons)
    steady_dq = checked_metric(summary, "steady_dq_rms_mean", reasons)
    tau_slew = checked_metric(summary, "tau_slew_rms_mean", reasons)
    saturation = checked_metric(summary, "saturation_ratio_max", reasons)
    actuator_lag = checked_metric(summary, "actuator_lag_max_abs", reasons)
    arm_contact = checked_metric(summary, "arm_contact_ratio", reasons)
    torque_margin = checked_metric(summary, "torque_margin_min_nm", reasons)
    if scenario.startswith("tool_"):
        tool_rms = checked_metric(summary, "tool_pos_err_rms_m", reasons)
        tool_max = checked_metric(summary, "tool_pos_err_max_m", reasons)
    else:
        tool_rms = 0.0
        tool_max = 0.0

    values = (
        q_rms,
        q_max,
        dq_rms,
        steady_dq,
        tau_slew,
        saturation,
        actuator_lag,
        arm_contact,
        torque_margin,
        tool_rms,
        tool_max,
    )
    if reasons or any(not math.isfinite(value) for value in values):
        return BIG_PENALTY, reasons

    loss = 0.0
    loss += 10.0 * (tool_rms / 0.04) ** 2
    loss += 4.0 * (tool_max / 0.12) ** 2
    loss += 2.0 * (q_rms / 0.08) ** 2
    loss += 1.4 * (q_max / 0.30) ** 2
    loss += 0.9 * (dq_rms / 0.60) ** 2
    loss += 1.8 * (steady_dq / 0.05) ** 2
    loss += 1.8 * (tau_slew / 180.0) ** 2
    loss += 24.0 * saturation
    loss += 5.0 * (actuator_lag / 14.0) ** 2
    loss += 5.0 * (max(0.0, 2.0 - torque_margin) / 2.0) ** 2
    loss += 70.0 * arm_contact
    return loss, reasons


def aggregate_loss(
    scenario_records: dict[str, dict[str, Any]],
    baseline_losses: dict[str, float] | None = None,
) -> tuple[float, dict[str, float], list[str]]:
    total = 0.0
    weight_sum = 0.0
    losses: dict[str, float] = {}
    reasons: list[str] = []
    regression_penalty = 0.0
    for scenario, record in scenario_records.items():
        loss, scenario_reasons = scenario_loss(scenario, record["summary"])
        weight = SCENARIO_WEIGHTS.get(scenario, 1.0)
        losses[scenario] = loss
        total += weight * loss
        weight_sum += weight
        if scenario_reasons:
            reasons.extend(f"{scenario}: {reason}" for reason in scenario_reasons)
        if baseline_losses is not None and scenario in baseline_losses:
            baseline_loss = baseline_losses[scenario]
            allowed_loss = baseline_loss * 1.08 + 0.10
            if math.isfinite(loss) and math.isfinite(baseline_loss) and loss > allowed_loss:
                penalty = 4.0 * weight * (loss - allowed_loss)
                regression_penalty += penalty
                losses[f"{scenario}__regression_penalty"] = penalty
    if weight_sum <= 0.0:
        return BIG_PENALTY, losses, ["no scenarios"]
    return (total + regression_penalty) / weight_sum, losses, reasons


def evaluate_candidate(
    root: Path,
    exe: Path,
    candidate_path: Path,
    override_path: Path,
    out_dir: Path,
    scenarios: tuple[str, ...],
    timeout_s: float,
    baseline_losses: dict[str, float] | None = None,
) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    out_dir.mkdir(parents=True, exist_ok=True)
    records: dict[str, dict[str, Any]] = {}
    for scenario in progress_iter(scenarios, f"{out_dir.name} scenarios", unit="case", leave=False):
        records[scenario] = run_one_scenario(root, exe, scenario, candidate_path, override_path, out_dir, timeout_s)
    loss, scenario_losses, reasons = aggregate_loss(records, baseline_losses)
    return {"loss": loss, "scenario_losses": scenario_losses, "failure_reasons": reasons}, records


def write_candidate_files(
    candidate_dir: Path,
    index: int,
    config: dict[str, Any],
) -> tuple[Path, Path]:
    candidate_path = candidate_dir / f"candidate_{index:03d}.yaml"
    override_path = candidate_dir / f"candidate_{index:03d}.params"
    write_yaml(candidate_path, config)
    write_override_file(override_path, config)
    return candidate_path, override_path


def evaluate_and_record(
    root: Path,
    exe: Path,
    run_dir: Path,
    candidate_dir: Path,
    index: int,
    stage: str,
    config: dict[str, Any],
    scenarios: tuple[str, ...],
    timeout_s: float,
    baseline_losses: dict[str, float] | None,
) -> dict[str, Any]:
    candidate_path, override_path = write_candidate_files(candidate_dir, index, config)
    out_dir = run_dir / f"candidate_{index:03d}"
    record: dict[str, Any] = {
        "candidate": index,
        "stage": stage,
        "config": str(candidate_path),
        "overrides": str(override_path),
        "params": params_snapshot(config),
        "design_scales": config.get("tuning", {}).get("design_scales", {}),
    }
    print(f"[candidate {index:03d}] {stage}", flush=True)
    try:
        result, metrics = evaluate_candidate(
            root,
            exe,
            candidate_path,
            override_path,
            out_dir,
            scenarios,
            timeout_s,
            baseline_losses,
        )
        record.update(result)
        record["metrics"] = metrics
        record["failed"] = bool(result["failure_reasons"])
    except Exception as exc:
        record["loss"] = BIG_PENALTY
        record["scenario_losses"] = {}
        record["failure_reasons"] = [str(exc)]
        record["metrics"] = {}
        record["failed"] = True
        print(f"  failed: {exc}", file=sys.stderr, flush=True)
    return record


def write_ranking(run_dir: Path, records: list[dict[str, Any]]) -> None:
    ranked = sorted(records, key=lambda item: item.get("loss", BIG_PENALTY))
    (run_dir / "ranking.json").write_text(json.dumps(ranked, indent=2), encoding="utf-8")


def metric_compact(record: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for scenario, result in record.get("metrics", {}).items():
        summary = result.get("summary", {})
        out[scenario] = {
            "q_err_rms_mean": summary.get("q_err_rms_mean"),
            "q_err_max_abs": summary.get("q_err_max_abs"),
            "tool_pos_err_rms_m": summary.get("tool_pos_err_rms_m"),
            "tool_pos_err_max_m": summary.get("tool_pos_err_max_m"),
            "steady_dq_rms_mean": summary.get("steady_dq_rms_mean"),
            "tau_slew_rms_mean": summary.get("tau_slew_rms_mean"),
            "saturation_ratio_max": summary.get("saturation_ratio_max"),
            "arm_contact_ratio": summary.get("arm_contact_ratio"),
        }
    return out


def scenario_delta_lines(baseline: dict[str, Any], best: dict[str, Any]) -> list[str]:
    lines = ["| scenario | baseline loss | best loss | delta |", "| --- | ---: | ---: | ---: |"]
    baseline_losses = baseline.get("scenario_losses", {})
    best_losses = best.get("scenario_losses", {})
    for scenario in sorted(set(baseline_losses) | set(best_losses)):
        if "__" in scenario:
            continue
        base_loss = float(baseline_losses.get(scenario, math.nan))
        best_loss = float(best_losses.get(scenario, math.nan))
        delta = best_loss - base_loss if math.isfinite(base_loss) and math.isfinite(best_loss) else math.nan
        lines.append(f"| {scenario} | {base_loss:.6g} | {best_loss:.6g} | {delta:+.6g} |")
    return lines


def write_summary(run_dir: Path, records: list[dict[str, Any]], baseline: dict[str, Any], model_seed: dict[str, Any], best: dict[str, Any]) -> None:
    ranked = sorted(records, key=lambda item: item.get("loss", BIG_PENALTY))
    baseline_loss = float(baseline.get("loss", BIG_PENALTY))
    seed_loss = float(model_seed.get("loss", BIG_PENALTY))
    best_loss = float(best.get("loss", BIG_PENALTY))
    improved = best_loss < baseline_loss
    improvement = 0.0 if not math.isfinite(baseline_loss) or baseline_loss <= 0.0 else 100.0 * (baseline_loss - best_loss) / baseline_loss

    lines = [
        "# Tuning Summary",
        "",
        f"- baseline loss: `{baseline_loss:.6g}`",
        f"- model seed loss: `{seed_loss:.6g}`",
        f"- best loss: `{best_loss:.6g}`",
        f"- improvement vs baseline: `{improvement:.2f}%`",
        f"- result: `{'improved' if improved else 'no improvement'}`",
        "",
        "## Top Candidates",
        "",
        "| rank | candidate | stage | loss |",
        "| --- | --- | --- | ---: |",
    ]
    for rank, item in enumerate(ranked[:10], start=1):
        lines.append(f"| {rank} | {item['candidate']} | {item['stage']} | {float(item['loss']):.6g} |")

    lines.extend(["", "## Scenario Loss Deltas", ""])
    lines.extend(scenario_delta_lines(baseline, best))
    lines.extend(["", "## Best Tuned Parameters", "", "```json"])
    lines.append(json.dumps(best.get("params", {}), indent=2))
    lines.extend(["```", "", "## Best Design Scales", "", "```json"])
    lines.append(json.dumps(best.get("design_scales", {}), indent=2))
    lines.extend(["```", ""])

    if best.get("failure_reasons"):
        lines.extend(["## Best Candidate Warnings", ""])
        for reason in best["failure_reasons"]:
            lines.append(f"- {reason}")
        lines.append("")

    lines.extend(["## Baseline Metrics", "", "```json"])
    lines.append(json.dumps(metric_compact(baseline), indent=2))
    lines.extend(["```", "", "## Best Metrics", "", "```json"])
    lines.append(json.dumps(metric_compact(best), indent=2))
    lines.extend(["```", ""])

    check = best.get("check_metrics")
    if check:
        lines.extend(["## Best Check-Only Metrics", "", "```json"])
        lines.append(json.dumps({k: v.get("summary", {}) for k, v in check.items()}, indent=2))
        lines.extend(["```", ""])

    (run_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def random_vector(rng: random.Random) -> list[float]:
    return [rng.gauss(0.0, 0.65) for _ in DESIGN_VARIABLES]


def evaluate_random_candidates(
    root: Path,
    exe: Path,
    run_dir: Path,
    candidate_dir: Path,
    start_index: int,
    count: int,
    seed_config: dict[str, Any],
    scenarios: tuple[str, ...],
    timeout_s: float,
    baseline_losses: dict[str, float] | None,
    seed: int,
    records: list[dict[str, Any]],
) -> None:
    rng = random.Random(seed)
    for offset in progress_iter(range(count), "candidates", unit="candidate"):
        index = start_index + offset
        config = make_scaled_candidate(index, "random", seed_config, random_vector(rng))
        record = evaluate_and_record(
            root, exe, run_dir, candidate_dir, index, "random", config, scenarios, timeout_s, baseline_losses
        )
        records.append(record)
        write_ranking(run_dir, records)


def evaluate_cma_candidates(
    root: Path,
    exe: Path,
    run_dir: Path,
    candidate_dir: Path,
    start_index: int,
    count: int,
    seed_config: dict[str, Any],
    scenarios: tuple[str, ...],
    timeout_s: float,
    baseline_losses: dict[str, float] | None,
    seed: int,
    records: list[dict[str, Any]],
) -> None:
    if count <= 0:
        return
    if count < 4:
        print("budget after baseline/model_seed is too small for CMA-style ES; using random high-level search.", file=sys.stderr)
        evaluate_random_candidates(
            root, exe, run_dir, candidate_dir, start_index, count, seed_config, scenarios, timeout_s, baseline_losses, seed, records
        )
        return

    rng = random.Random(seed)
    dim = len(DESIGN_VARIABLES)
    popsize = min(8, max(4, count))
    mean = [0.0] * dim
    sigma = [0.55] * dim
    evaluated = 0
    with_progress = progress_iter(range(count), "candidates", unit="candidate")
    progress_iterator = iter(with_progress)
    while evaluated < count:
        remaining = count - evaluated
        batch_count = min(popsize, remaining)
        asked = [[mean[i] + sigma[i] * rng.gauss(0.0, 1.0) for i in range(dim)] for _ in range(batch_count)]
        batch_vectors: list[list[float]] = []
        batch_losses: list[float] = []
        for vector in asked:
            if evaluated >= count:
                break
            next(progress_iterator, None)
            index = start_index + evaluated
            config = make_scaled_candidate(index, "cma", seed_config, vector)
            record = evaluate_and_record(
                root, exe, run_dir, candidate_dir, index, "cma", config, scenarios, timeout_s, baseline_losses
            )
            records.append(record)
            write_ranking(run_dir, records)
            batch_vectors.append(vector)
            batch_losses.append(float(record.get("loss", BIG_PENALTY)))
            evaluated += 1
        if len(batch_vectors) >= 2:
            ranked = sorted(zip(batch_losses, batch_vectors), key=lambda item: item[0])
            elite_count = max(2, len(ranked) // 2)
            elites = [vector for _, vector in ranked[:elite_count]]
            elite_weights = [elite_count - i for i in range(elite_count)]
            weight_sum = float(sum(elite_weights))
            elite_mean = [
                sum(weight * vector[i] for weight, vector in zip(elite_weights, elites)) / weight_sum
                for i in range(dim)
            ]
            elite_sigma = []
            for i in range(dim):
                variance = sum(weight * (vector[i] - elite_mean[i]) ** 2 for weight, vector in zip(elite_weights, elites)) / weight_sum
                elite_sigma.append(clamp(math.sqrt(max(variance, 1.0e-6)), 0.12, 1.10))
            mean = [0.65 * mean[i] + 0.35 * elite_mean[i] for i in range(dim)]
            sigma = [0.75 * sigma[i] + 0.25 * elite_sigma[i] for i in range(dim)]


def main() -> int:
    global VERBOSE_COMMANDS
    args = parse_args()
    VERBOSE_COMMANDS = bool(args.verbose_commands)

    root = root_dir()
    config_path = rel_or_abs(args.config, root)
    build_dir = rel_or_abs(args.build_dir, root)
    out_root = rel_or_abs(args.out_root, root)
    main_scenarios = tuple(args.scenario) if args.scenario else DEFAULT_MAIN_SCENARIOS
    check_scenarios = tuple(args.check_scenario) if args.check_scenario else (() if args.scenario else DEFAULT_CHECK_SCENARIOS)

    if args.profile != "teleop_core":
        print(f"unsupported profile: {args.profile}", file=sys.stderr)
        return 2
    if args.budget < 2:
        print("--budget must be at least 2 because candidate 0 is baseline and candidate 1 is model_seed.", file=sys.stderr)
        return 2
    if not args.allow_dirty:
        dirty = git_tracked_dirty(root)
        if dirty:
            print("Tracked worktree is dirty; commit/stash changes or pass --allow-dirty.", file=sys.stderr)
            print(dirty, file=sys.stderr)
            return 2

    exe = benchmark_exe(build_dir)
    if not exe.exists():
        print(f"Benchmark executable not found: {exe}. Run `cmake --build build` first.", file=sys.stderr)
        return 2

    timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = out_root / timestamp
    candidate_dir = run_dir / "candidates"
    candidate_dir.mkdir(parents=True, exist_ok=False)

    original_config = load_yaml(config_path)
    diagnostics = sample_model_diagnostics(original_config)
    seed_config = build_model_seed_config(original_config, diagnostics)
    write_yaml(run_dir / "seed_design.yaml", seed_config)
    (run_dir / "seed_diagnostics.json").write_text(json.dumps(diagnostics, indent=2), encoding="utf-8")

    records: list[dict[str, Any]] = []
    baseline_losses: dict[str, float] | None = None

    baseline_config = copy.deepcopy(original_config)
    baseline_config["tuning"] = {"candidate": 0, "stage": "baseline"}
    baseline_record = evaluate_and_record(
        root, exe, run_dir, candidate_dir, 0, "baseline", baseline_config, main_scenarios, args.timeout_s, None
    )
    records.append(baseline_record)
    baseline_losses = {
        key: value
        for key, value in baseline_record.get("scenario_losses", {}).items()
        if "__" not in key and math.isfinite(float(value))
    }
    write_ranking(run_dir, records)

    model_seed_record = evaluate_and_record(
        root, exe, run_dir, candidate_dir, 1, "model_seed", seed_config, main_scenarios, args.timeout_s, baseline_losses
    )
    records.append(model_seed_record)
    write_ranking(run_dir, records)

    remaining = args.budget - 2
    print(f"optimizer: {args.optimizer}", flush=True)
    if args.optimizer == "cma":
        evaluate_cma_candidates(
            root,
            exe,
            run_dir,
            candidate_dir,
            2,
            remaining,
            seed_config,
            main_scenarios,
            args.timeout_s,
            baseline_losses,
            args.seed,
            records,
        )
    else:
        evaluate_random_candidates(
            root,
            exe,
            run_dir,
            candidate_dir,
            2,
            remaining,
            seed_config,
            main_scenarios,
            args.timeout_s,
            baseline_losses,
            args.seed,
            records,
        )

    ranked = sorted(records, key=lambda item: item.get("loss", BIG_PENALTY))
    best_record = ranked[0]
    best_path = run_dir / "best.yaml"
    shutil.copy2(best_record["config"], best_path)
    shutil.copy2(best_record["overrides"], run_dir / "best.params")

    if check_scenarios:
        _, baseline_check = evaluate_candidate(
            root,
            exe,
            Path(baseline_record["config"]),
            Path(baseline_record["overrides"]),
            run_dir / "candidate_000_check",
            check_scenarios,
            args.timeout_s,
        )
        if best_record["candidate"] == baseline_record["candidate"]:
            best_check = baseline_check
        else:
            _, best_check = evaluate_candidate(
                root,
                exe,
                Path(best_record["config"]),
                Path(best_record["overrides"]),
                run_dir / f"candidate_{best_record['candidate']:03d}_check",
                check_scenarios,
                args.timeout_s,
            )
        baseline_record["check_metrics"] = baseline_check
        best_record["check_metrics"] = best_check
        write_ranking(run_dir, records)

    write_summary(run_dir, records, baseline_record, model_seed_record, best_record)
    print(f"best: candidate_{best_record['candidate']:03d} loss={float(best_record['loss']):.6g}")
    print(f"wrote: {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
