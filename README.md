# ArmSim

ArmSim is split into a portable control core and a MuJoCo simulation harness.

## Layout

- `arm_core/`: pure C reusable arm core, including interfaces, reference sources, controllers, safety, and verification helpers. This is the part intended to be synced into a Keil/STM32F4xx project.
- `sim_mujoco/`: MuJoCo adapters, placeholder model, viewer, headless verifier, and CSV logging.
- `configs/`: human-maintained arm model and tuning source files. The current placeholder arm is defined by `configs/arm6_placeholder.yaml`.
- `tools/`: PC-side generation and analysis tools. These are not part of the embedded core.
- `third_party/`: external PC-side dependencies such as MuJoCo and GLFW.
- `reference/`: read-only reference code. It is not built or modified by this project.

## Config Generation

The placeholder arm uses `configs/arm6_placeholder.yaml` as the single source
for joint order, axes, limits, actuator names, controller tuning, simulation
impairments, and MuJoCo geometry. Generated files are committed so normal CMake
builds do not need Python or PyYAML:

- `sim_mujoco/models/arm6_placeholder.xml`
- `sim_mujoco/include/armsim/arm6_sim_config.h`
- `sim_mujoco/src/default_arm_config.c`

After editing the config, regenerate the files:

```powershell
python tools/generate_arm_model.py --config configs/arm6_placeholder.yaml
```

To check whether generated files are stale:

```powershell
python tools/generate_arm_model.py --config configs/arm6_placeholder.yaml --check
```

The generator reads YAML through PyYAML and also supports JSON input. The
generated C and XML files are tool outputs; change the config instead of editing
them by hand. If PyYAML is missing, install it with:

```powershell
python -m pip install PyYAML
```

## Build

The control core can be configured without MuJoCo:

```powershell
cmake -S . -B build -G Ninja -DARMSIM_BUILD_SIM=OFF
cmake --build build
ctest --test-dir build
```

To build MuJoCo targets, use:

```powershell
git submodule update --init --recursive
cmake -S . -B build -G Ninja -DARMSIM_BUILD_SIM=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

`third_party/glfw` is a Git submodule pinned to GLFW 3.4. `third_party/mujoco`
is a local, ignored MuJoCo 3.9.0 prebuilt SDK directory because the official
Windows binary release is not itself a Git repository. If MuJoCo is missing,
CMake will skip the unavailable simulation targets with a warning.

## Simulation Programs

- `armsim_verify_joints`: headless joint torque I/O verification, writes `logs/joint_verify.csv`.
- `armsim_control_benchmark`: headless harsh-sim control benchmark, writes logs such as `logs/control_benchmark_hold_zero_harsh.csv`.
- `armsim_viewer`: MuJoCo viewer for visual joint direction checks, joint target editing, and Dynamic-mode `Tool Drag` end-effector position dragging.

The default model is `sim_mujoco/models/arm6_placeholder.xml`.

In `armsim_viewer`, enable `Tool Drag` with the panel button or `T`, switch to
`Dynamic`, choose `X`, `Y`, or `Z` in the panel or with the matching key, then
left-drag in the scene to move the `tool0` target along that world axis. The
drag target keeps the `tool0` orientation captured at mouse press. Releasing the
mouse syncs the target back to the current joint pose so the arm holds
immediately instead of continuing toward a stale Cartesian target.

## Control Metrics

Control logs use a source-agnostic joint-space CSV schema. The analyzer needs
`time_s`, `q_refN`, `dq_refN`, `tau_cmdN`, and filtered state columns
`q_filtN/dq_filtN`; it also accepts `qN/dqN` aliases for simpler real-arm logs.
Optional columns such as `q_measN`, `dq_measN`, `tau_ff_gravityN`, `tau_fbN`,
`mj_ctrlN`, `state_flags`, and `command_flags` enrich the report but are not
required.

Run a benchmark:

```powershell
.\build\sim_mujoco\armsim_control_benchmark.exe hold_zero_harsh logs/control_benchmark_hold_zero_harsh.csv
```

Analyze a log:

```powershell
python tools/analyze_control_log.py logs/control_benchmark_hold_zero_harsh.csv --config configs/arm6_placeholder.yaml
```

Run all default benchmarks and analyze them in one pass:

```powershell
python tools/run_control_benchmarks.py
```

The analyzer is not tied to MuJoCo. A future real-arm miniPC logger can reuse it
by recording compatible CSV rows from serial samples; JSONL can be added later
without changing the core metrics definitions.

Keil/STM32 sync tooling will be added after the real-vehicle adapter exists.
