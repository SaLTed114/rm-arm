#!/usr/bin/env python3
"""Generate ArmSim MuJoCo and C config files from one arm model config."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


GENERATED_NOTE = "Generated from {config}. Do not edit by hand."


def load_config(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    suffix = path.suffix.lower()
    if suffix in (".yaml", ".yml"):
        try:
            import yaml  # type: ignore
        except ModuleNotFoundError:
            try:
                data = json.loads(text)
            except json.JSONDecodeError as exc:
                raise SystemExit(
                    f"{path} is YAML, but PyYAML is not installed and the file is not JSON-compatible YAML.\n"
                    "Install PyYAML or pass a .json config."
                ) from exc
        else:
            data = yaml.safe_load(text)
    else:
        data = json.loads(text)

    if not isinstance(data, dict):
        raise SystemExit(f"{path} must contain a top-level object")
    return data


def fmt_num(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:.12g}"
    return str(value)


def xml_value(value: Any) -> str:
    if isinstance(value, list):
        return " ".join(fmt_num(item) for item in value)
    return fmt_num(value)


def xml_attrs(attrs: dict[str, Any]) -> str:
    return " ".join(f'{key}="{xml_value(value)}"' for key, value in attrs.items())


def xml_line(lines: list[str], indent: int, tag: str, attrs: dict[str, Any]) -> None:
    lines.append(f"{'  ' * indent}<{tag} {xml_attrs(attrs)}/>")


def joint_lookup(config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {joint["name"]: joint for joint in config["joints"]}


def append_joint(lines: list[str], indent: int, joint_name: str, joints: dict[str, dict[str, Any]], default_range: list[float]) -> None:
    joint = joints[joint_name]
    attrs: dict[str, Any] = {
        "name": joint["name"],
        "axis": joint["axis"],
    }
    joint_range = [joint["q_min_rad"], joint["q_max_rad"]]
    if joint_range != default_range:
        attrs["range"] = joint_range
    xml_line(lines, indent, "joint", attrs)


def append_body(lines: list[str], indent: int, body: dict[str, Any], joints: dict[str, dict[str, Any]], default_range: list[float]) -> None:
    attrs = {"name": body["name"], "pos": body["pos"]}
    lines.append(f"{'  ' * indent}<body {xml_attrs(attrs)}>")
    if "joint" in body:
        append_joint(lines, indent + 1, body["joint"], joints, default_range)
    for geom in body.get("geoms", []):
        xml_line(lines, indent + 1, "geom", geom)
    for site in body.get("sites", []):
        xml_line(lines, indent + 1, "site", site)
    for child in body.get("bodies", []):
        lines.append("")
        append_body(lines, indent + 1, child, joints, default_range)
    lines.append(f"{'  ' * indent}</body>")


def generate_mjcf(config: dict[str, Any], config_path: str) -> str:
    mujoco = config["mujoco"]
    joints = joint_lookup(config)
    default_range = mujoco["defaults"]["joint"]["range"]
    lines: list[str] = [
        f"<!-- {GENERATED_NOTE.format(config=config_path)} -->",
        f'<mujoco model="{config["model"]}">',
    ]
    xml_line(lines, 1, "compiler", mujoco["compiler"])
    xml_line(lines, 1, "option", mujoco["option"])
    lines.append("")
    lines.append("  <default>")
    xml_line(lines, 2, "joint", mujoco["defaults"]["joint"])
    xml_line(lines, 2, "geom", mujoco["defaults"]["geom"])
    xml_line(lines, 2, "motor", mujoco["defaults"]["motor"])
    lines.append("  </default>")
    lines.append("")
    lines.append("  <worldbody>")
    for light in mujoco["worldbody"].get("lights", []):
        xml_line(lines, 2, "light", light)
    for geom in mujoco["worldbody"].get("geoms", []):
        xml_line(lines, 2, "geom", geom)
    lines.append("")
    for body in mujoco["worldbody"].get("bodies", []):
        append_body(lines, 2, body, joints, default_range)
    lines.append("  </worldbody>")
    lines.append("")
    lines.append("  <actuator>")
    for joint in config["joints"]:
        limit = joint["torque_limit_nm"]
        attrs = {
            "name": joint["actuator"],
            "joint": joint["name"],
            "ctrlrange": [-limit, limit],
        }
        xml_line(lines, 2, "motor", attrs)
    lines.append("  </actuator>")
    lines.append("</mujoco>")
    return "\n".join(lines) + "\n"


def c_real(value: Any) -> str:
    if isinstance(value, str):
        return value
    numeric = float(value)
    if numeric == 0.0:
        return "ARM_REAL_ZERO"
    if numeric == 1.0:
        return "ARM_REAL_ONE"
    return f"ARM_REAL({fmt_num(numeric)})"


def c_array(values: list[Any]) -> str:
    return "{ " + ", ".join(c_real(value) for value in values) + " }"


def c_bool(value: bool) -> str:
    return "true" if value else "false"


def vec3(value: Any | None) -> list[float]:
    if value is None:
        return [0.0, 0.0, 0.0]
    if not isinstance(value, list) or len(value) != 3:
        raise SystemExit(f"expected vec3, got {value!r}")
    return [float(value[0]), float(value[1]), float(value[2])]


def geom_center(geom: dict[str, Any]) -> list[float]:
    if "pos" in geom:
        return vec3(geom["pos"])
    if "fromto" in geom:
        values = geom["fromto"]
        if not isinstance(values, list) or len(values) != 6:
            raise SystemExit(f"invalid fromto on geom {geom.get('name', '<unnamed>')}")
        return [
            0.5 * (float(values[0]) + float(values[3])),
            0.5 * (float(values[1]) + float(values[4])),
            0.5 * (float(values[2]) + float(values[5])),
        ]
    return [0.0, 0.0, 0.0]


def body_mass_and_com(body: dict[str, Any]) -> tuple[float, list[float]]:
    total_mass = 0.0
    weighted = [0.0, 0.0, 0.0]
    for geom in body.get("geoms", []):
        mass = float(geom.get("mass", 0.0))
        if mass <= 0.0:
            continue
        center = geom_center(geom)
        total_mass += mass
        for axis in range(3):
            weighted[axis] += mass * center[axis]
    if total_mass <= 0.0:
        return 0.0, [0.0, 0.0, 0.0]
    return total_mass, [weighted[axis] / total_mass for axis in range(3)]


def gravity_bodies(config: dict[str, Any]) -> list[dict[str, Any]]:
    joint_indices = {joint["name"]: index for index, joint in enumerate(config["joints"])}
    joints = joint_lookup(config)
    bodies: list[dict[str, Any]] = []

    def visit(body: dict[str, Any], parent: int) -> None:
        index = len(bodies)
        joint_name = body.get("joint")
        joint_index = joint_indices[joint_name] if joint_name else -1
        axis = joints[joint_name]["axis"] if joint_name else [0, 0, 0]
        mass, com = body_mass_and_com(body)
        bodies.append(
            {
                "parent": parent,
                "joint": joint_index,
                "pos": body["pos"],
                "axis": axis,
                "mass": mass,
                "com": com,
            }
        )
        for child in body.get("bodies", []):
            visit(child, index)

    for root_body in config["mujoco"]["worldbody"].get("bodies", []):
        visit(root_body, -1)
    return bodies


def kinematics_bodies(config: dict[str, Any], tool_site: str) -> tuple[list[dict[str, Any]], int, list[float]]:
    joint_indices = {joint["name"]: index for index, joint in enumerate(config["joints"])}
    joints = joint_lookup(config)
    bodies: list[dict[str, Any]] = []
    tool_body = -1
    tool_pos = [0.0, 0.0, 0.0]

    def visit(body: dict[str, Any], parent: int) -> None:
        nonlocal tool_body, tool_pos
        index = len(bodies)
        joint_name = body.get("joint")
        joint_index = joint_indices[joint_name] if joint_name else -1
        axis = joints[joint_name]["axis"] if joint_name else [0, 0, 0]
        q_min = joints[joint_name]["q_min_rad"] if joint_name else 0.0
        q_max = joints[joint_name]["q_max_rad"] if joint_name else 0.0
        bodies.append(
            {
                "parent": parent,
                "joint": joint_index,
                "pos": body["pos"],
                "axis": axis,
                "q_min": q_min,
                "q_max": q_max,
            }
        )
        for site in body.get("sites", []):
            if site.get("name") == tool_site:
                tool_body = index
                tool_pos = vec3(site.get("pos"))
        for child in body.get("bodies", []):
            visit(child, index)

    for root_body in config["mujoco"]["worldbody"].get("bodies", []):
        visit(root_body, -1)
    if tool_body < 0:
        raise SystemExit(f"tool site {tool_site!r} was not found")
    return bodies, tool_body, tool_pos


def c_body_index(value: int, none_macro: str) -> str:
    return none_macro if value < 0 else f"{value}"


def generate_sim_config_header(config: dict[str, Any], config_path: str) -> str:
    control = config["control"]
    harsh = config["harsh_sim"]
    torque_limits = [joint["torque_limit_nm"] for joint in config["joints"]]
    lines = [
        f"/* {GENERATED_NOTE.format(config=config_path)} */",
        "#ifndef ARMSIM_ARM6_SIM_CONFIG_H_",
        "#define ARMSIM_ARM6_SIM_CONFIG_H_",
        "",
        '#include "arm_core/joint_gravity_ff.h"',
        '#include "arm_motion/joint_kinematics.h"',
        '#include "arm_core/arm_types.h"',
        "",
        "#define ARMSIM_ARM6_TORQUE_LIMITS_NM \\",
        f"  {c_array(torque_limits)}",
        "",
        "#define ARMSIM_ARM6_PD_PARAMS \\",
        "  { \\",
    ]
    pd_rows = []
    for params in control["pd"]:
        values = [params["kp"], params["kd"], params["out_limit"]]
        pd_rows.append(f"    {c_array(values)}")
    for index, row in enumerate(pd_rows):
        suffix = " \\" if index != len(pd_rows) - 1 else " \\"
        comma = "," if index != len(pd_rows) - 1 else ""
        lines.append(f"{row}{comma}{suffix}")
    lines.extend([
        "  }",
        "",
        "#define ARMSIM_ARM6_DQ_LIMITS_RAD_S \\",
        f"  {c_array(control['reference']['dq_limits_rad_s'])}",
        "",
        "#define ARMSIM_ARM6_DDQ_LIMITS_RAD_S2 \\",
        f"  {c_array(control['reference']['ddq_limits_rad_s2'])}",
        "",
        "#define ARMSIM_ARM6_STATE_FILTER_PARAMS \\",
        "  { \\",
    ])
    filter_rows = []
    for params in control["state_filter"]:
        row = "{ " + ", ".join(
            [
                c_real(params["q_time_constant_s"]),
                c_real(params["dq_time_constant_s"]),
                c_bool(params["use_dq_from_q_diff"]),
            ]
        ) + " }"
        filter_rows.append(f"    {row}")
    for index, row in enumerate(filter_rows):
        suffix = " \\" if index != len(filter_rows) - 1 else " \\"
        comma = "," if index != len(filter_rows) - 1 else ""
        lines.append(f"{row}{comma}{suffix}")
    body_rows = []
    for body in gravity_bodies(config):
        row = (
            "    { "
            + ", ".join(
                [
                    c_body_index(body["parent"], "JOINT_GRAVITY_FF_NO_BODY"),
                    c_body_index(body["joint"], "JOINT_GRAVITY_FF_NO_JOINT"),
                    c_array(body["pos"]),
                    c_array(body["axis"]),
                    c_real(body["mass"]),
                    c_array(body["com"]),
                ]
            )
            + " }"
        )
        body_rows.append(row)
    gravity = config["mujoco"]["option"]["gravity"]
    lines.extend([
        "  }",
        "",
        "#define ARMSIM_ARM6_GRAVITY_FF_PARAMS \\",
        "  { \\",
        "    ARM_DEFAULT_DOF, \\",
        f"    {len(body_rows)}u, \\",
        f"    {c_array(gravity)}, \\",
        "    { \\",
    ])
    for index, row in enumerate(body_rows):
        suffix = " \\" if index != len(body_rows) - 1 else " \\"
        comma = "," if index != len(body_rows) - 1 else ""
        lines.append(f"{row}{comma}{suffix}")
    kinematics, tool_body, tool_pos = kinematics_bodies(config, "tool0")
    kin_rows = []
    for body in kinematics:
        row = (
            "    { "
            + ", ".join(
                [
                    c_body_index(body["parent"], "JOINT_KINEMATICS_NO_BODY"),
                    c_body_index(body["joint"], "JOINT_KINEMATICS_NO_JOINT"),
                    c_array(body["pos"]),
                    c_array(body["axis"]),
                    c_real(body["q_min"]),
                    c_real(body["q_max"]),
                ]
            )
            + " }"
        )
        kin_rows.append(row)
    lines.extend([
        "    } \\",
        "  }",
        "",
        "#define ARMSIM_ARM6_KINEMATICS_PARAMS \\",
        "  { \\",
        "    ARM_DEFAULT_DOF, \\",
        f"    {len(kin_rows)}u, \\",
        f"    {tool_body}, \\",
        f"    {c_array(tool_pos)}, \\",
        "    { \\",
    ])
    for index, row in enumerate(kin_rows):
        suffix = " \\" if index != len(kin_rows) - 1 else " \\"
        comma = "," if index != len(kin_rows) - 1 else ""
        lines.append(f"{row}{comma}{suffix}")
    lines.extend([
        "    } \\",
        "  }",
        "",
        f"#define ARMSIM_ARM6_SAFETY_Q_MARGIN_RAD {c_real(control['safety']['q_margin_rad'])}",
        f"#define ARMSIM_ARM6_SAFETY_DQ_LIMIT_SCALE {c_real(control['safety']['dq_limit_scale'])}",
        "",
        f"#define ARMSIM_HARSH_CONTROL_PERIOD_S {c_real(harsh['control_period_s'])}",
        f"#define ARMSIM_HARSH_CONTROL_JITTER_S {c_real(harsh['control_jitter_s'])}",
        f"#define ARMSIM_HARSH_SENSOR_DELAY_STEPS {int(harsh['sensor_delay_steps'])}u",
        f"#define ARMSIM_HARSH_ENCODER_RESOLUTION_RAD {c_real(harsh['encoder_resolution_rad'])}",
        f"#define ARMSIM_HARSH_Q_NOISE_RAD {c_real(harsh['q_noise_rad'])}",
        f"#define ARMSIM_HARSH_DQ_NOISE_RAD_S {c_real(harsh['dq_noise_rad_s'])}",
        f"#define ARMSIM_HARSH_TAU_EST_NOISE_NM {c_real(harsh['tau_est_noise_nm'])}",
        f"#define ARMSIM_HARSH_DQ_FILTER_ALPHA {c_real(harsh['dq_filter_alpha'])}",
        f"#define ARMSIM_HARSH_ACTUATOR_TAU_TIME_CONSTANT_S {c_real(harsh['actuator_tau_time_constant_s'])}",
        f"#define ARMSIM_HARSH_ACTUATOR_TAU_RATE_LIMIT_NM_S {c_real(harsh['actuator_tau_rate_limit_nm_s'])}",
        f"#define ARMSIM_HARSH_ACTUATOR_DEADBAND_NM {c_real(harsh['actuator_deadband_nm'])}",
        f"#define ARMSIM_HARSH_RANDOM_SEED {harsh['random_seed']}u",
        "",
        "#endif",
    ])
    return "\n".join(lines) + "\n"


def generate_default_arm_config(config: dict[str, Any], config_path: str) -> str:
    lines = [
        f"/* {GENERATED_NOTE.format(config=config_path)} */",
        '#include "armsim/default_arm_config.h"',
        "",
        '#include "armsim/arm6_sim_config.h"',
        "",
        "arm_config_t armsim_default_arm6_config(void) {",
        "  arm_config_t config = {0};",
        "  static const arm_real_t torque_limits[ARM_DEFAULT_DOF] = ARMSIM_ARM6_TORQUE_LIMITS_NM;",
        "  config.dof = ARM_DEFAULT_DOF;",
        "",
    ]
    for index, joint in enumerate(config["joints"]):
        lines.append(
            f'  config.joints[{index}] = (arm_joint_config_t){{"{joint["name"]}", "{joint["actuator"]}", '
            f'{c_real(joint["sign"])}, {c_real(joint["q_offset_rad"])}, torque_limits[{index}]}};'
        )
    lines.extend([
        "",
        "  return config;",
        "}",
    ])
    return "\n".join(lines) + "\n"


def source_label(config: dict[str, Any], config_path: Path, root: Path) -> str:
    if "source_label" in config:
        return str(config["source_label"])
    try:
        return config_path.resolve().relative_to(root).as_posix()
    except ValueError:
        return config_path.resolve().as_posix()


def generated_files(config: dict[str, Any], root: Path, config_label: str) -> dict[Path, str]:
    outputs = config["outputs"]
    return {
        root / outputs["mjcf"]: generate_mjcf(config, config_label),
        root / outputs["sim_config_header"]: generate_sim_config_header(config, config_label),
        root / outputs["default_arm_config"]: generate_default_arm_config(config, config_label),
    }


def write_or_check(files: dict[Path, str], check: bool) -> int:
    failed = False
    for path, content in files.items():
        if check:
            try:
                existing = path.read_text(encoding="utf-8")
            except FileNotFoundError:
                print(f"missing generated file: {path}", file=sys.stderr)
                failed = True
                continue
            if existing != content:
                print(f"generated file is stale: {path}", file=sys.stderr)
                failed = True
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8", newline="\n")
            print(f"generated {path}")
    return 1 if failed else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/arm6_placeholder.yaml", help="YAML or JSON arm config")
    parser.add_argument("--root", default=None, help="Repository root; defaults to parent of this script")
    parser.add_argument("--check", action="store_true", help="Check generated files without writing")
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    root = Path(args.root).resolve() if args.root else script_dir.parent
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = root / config_path

    config = load_config(config_path)
    files = generated_files(config, root, source_label(config, config_path, root))
    return write_or_check(files, args.check)


if __name__ == "__main__":
    raise SystemExit(main())
