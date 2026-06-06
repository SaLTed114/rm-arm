# ArmSim

ArmSim is split into portable C modules and a MuJoCo simulation harness.

## Layout

- `arm_common/`: pure C shared numeric basics and math helpers used by core, motion, and simulation code.
- `arm_core/`: pure C reusable arm core for `state + reference -> command`, including interfaces, controllers, feedforward, estimation, safety, and verification helpers.
- `arm_motion/`: pure C reusable motion/reference generation code, including joint reference shaping and lightweight kinematics/IK.
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

The placeholder arm separates visual/inertial geometry from coarse collision
geometry. Visual geoms do not collide, while a small set of massless collision
geoms approximates the outer arm shape for floor and obstacle contact. This
avoids artificial self-contact while still preventing obvious floor tunneling.

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
`mj_ctrlN`, contact diagnostics, `state_flags`, and `command_flags` enrich the
report but are not required.

Run a benchmark:

```powershell
.\build\sim_mujoco\armsim_control_benchmark.exe hold_zero_harsh logs/control_benchmark_hold_zero_harsh.csv
```

Benchmark feedforward modes can be selected with `--ff=none`, `--ff=gravity`,
or `--ff=inverse`. `inverse` uses MuJoCo online inverse dynamics and is a
simulation oracle only.

To watch the exact benchmark case instead of running it headless, pass
`--gui=on`:

```powershell
.\build\sim_mujoco\armsim_control_benchmark.exe straight_arm_lift_harsh logs/straight_arm_lift_gui.csv --ff=gravity --harsh=on --contacts=off --gui=on
```

Benchmark diagnostics can also split the same motion across impaired and ideal
simulation conditions:

```powershell
python tools/run_control_benchmarks.py --scenario straight_arm_lift_harsh --ff gravity --ff inverse --harsh on --harsh off --contacts on --contacts off
```

`--harsh=off` disables the sensor/actuator impairment model. `--contacts=off`
disables MuJoCo contacts. Comparing those cases separates controller/feedforward
issues from actuator lag, torque saturation, and contact or placeholder-model
collision artifacts.

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

## Feedforward Models

`joint_gravity_ff` is a portable analytic gravity compensator in `arm_core`.
It is suitable for firmware once link masses and centers of mass are reliable.

`mujoco_inverse_dynamics_ff` lives in `sim_mujoco` and calls MuJoCo `mj_inverse`
online. It estimates the best-case full dynamics feedforward from
current `q/dq` plus `ddq_ref`, and should not be copied into firmware. Using
current state makes the oracle more stable for manual or teleop-style references
where tracking error can be nonzero.

`joint_id_fit_ff` is the portable fitted inverse-dynamics evaluator. Its
coefficients are generated from CSV samples. First sample inverse dynamics with
the C-side MuJoCo tool:

```powershell
.\build\sim_mujoco\armsim_sample_inverse_dynamics.exe logs/inverse_dynamics_samples.csv 2000
```

Then fit and generate coefficients with Python:

```powershell
python tools/fit_inverse_dynamics_ff.py --config configs/arm6_placeholder.yaml --fit --samples-csv logs/inverse_dynamics_samples.csv
```

Python does not need the MuJoCo package for fitting; MuJoCo is only used by the
C sampler. Without samples, the script can still check or generate the committed
zero-coefficient placeholder:

```powershell
python tools/fit_inverse_dynamics_ff.py --config configs/arm6_placeholder.yaml --check
```

Keil/STM32 sync tooling will be added after the real-vehicle adapter exists.
