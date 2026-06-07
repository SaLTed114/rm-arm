# ArmSim Command Set

Run commands from the repository root.

## Tool Environment

Create the optional Python tooling environment:

```powershell
conda env create -f environment.yml
conda activate armsim
```

This environment is for codegen, analysis, tuning, and future offline modeling
tools. The C/CMake build still uses the checked-in generated files directly.

## Generate

```powershell
python tools\generate_arm_model.py --config configs\arm6_placeholder.yaml
python tools\generate_arm_model.py --config configs\arm6_placeholder.yaml --check
```

## Build And Test

```powershell
cmake --build build
ctest --test-dir build --output-on-failure
```

## Viewer

```powershell
.\build\sim_mujoco\armsim_viewer.exe
```

## Benchmark

Run the default benchmark set:

```powershell
python tools\run_control_benchmarks.py
```

Run focused joint-space control cases:

```powershell
python tools\run_control_benchmarks.py --scenario step_j2_harsh --scenario step_j3_harsh --scenario coupled_j2j3_harsh --scenario sine_j2_harsh --scenario straight_arm_lift_harsh --ff gravity --harsh on --contacts on
```

Run a teleop-like q-only multi-joint case. This updates only raw joint position
goals and lets `joint_ref_shaper` generate `q/dq/ddq` for control:

```powershell
python tools\run_control_benchmarks.py --scenario teleop_wave_j2j3j5_harsh --ff gravity --harsh on --contacts on
```

Run task-space tool path cases:

```powershell
python tools\run_control_benchmarks.py --scenario tool_circle_xz_harsh --scenario tool_square_xz_harsh --scenario tool_insert_line_harsh --ff gravity --harsh on --contacts on
```

Run joint-space trajectory cases. These benchmark cases build an IK-derived
joint waypoint table at startup, then track only joint-space `q/dq/ddq`
references. They approximate the intended offline-IK/upstream-planner flow and
are the preferred control-tuning cases for the current task mix:

```powershell
python tools\run_control_benchmarks.py --scenario joint_circle_j2j3_harsh --scenario joint_square_j2j3_harsh --scenario joint_insert_line_harsh --ff gravity --harsh on --contacts on
```

Run all circle/square task-space planes:

```powershell
python tools\run_control_benchmarks.py --scenario tool_circle_xy_harsh --scenario tool_circle_xz_harsh --scenario tool_circle_yz_harsh --scenario tool_square_xy_harsh --scenario tool_square_xz_harsh --scenario tool_square_yz_harsh --ff gravity --harsh on --contacts on
```

Compare gravity and MuJoCo inverse dynamics feedforward:

```powershell
python tools\run_control_benchmarks.py --scenario straight_arm_lift_harsh --scenario tool_circle_xz_harsh --ff gravity --ff inverse --harsh on --contacts on
```

Open a benchmark GUI while running one case:

```powershell
.\build\sim_mujoco\armsim_control_benchmark.exe tool_circle_xz_harsh logs\control_benchmark_tool_circle_xz_harsh_gui.csv --ff=gravity --harsh=on --contacts=on --gui=on
```

Open IK-derived joint-space trajectory GUI cases:

```powershell
.\build\sim_mujoco\armsim_control_benchmark.exe joint_circle_j2j3_harsh logs\control_benchmark_joint_circle_j2j3_harsh_gui.csv --ff=gravity --harsh=on --contacts=on --gui=on
.\build\sim_mujoco\armsim_control_benchmark.exe joint_square_j2j3_harsh logs\control_benchmark_joint_square_j2j3_harsh_gui.csv --ff=gravity --harsh=on --contacts=on --gui=on
.\build\sim_mujoco\armsim_control_benchmark.exe joint_insert_line_harsh logs\control_benchmark_joint_insert_line_harsh_gui.csv --ff=gravity --harsh=on --contacts=on --gui=on
.\build\sim_mujoco\armsim_control_benchmark.exe teleop_wave_j2j3j5_harsh logs\control_benchmark_teleop_wave_j2j3j5_harsh_gui.csv --ff=gravity --harsh=on --contacts=on --gui=on
```

For benchmark GUI trails, cyan is the desired `tool0` path and orange is the
actual `tool0` path. Task-space cases draw the requested tool target directly;
joint-space trajectory cases draw the FK result of the requested joint
reference.

## Analyze Existing Logs

```powershell
python tools\analyze_control_log.py logs\control_benchmark_tool_circle_xz_harsh_gravity_harsh-on_contacts-on.csv --config configs\arm6_placeholder.yaml
```

Write metrics JSON:

```powershell
python tools\analyze_control_log.py logs\control_benchmark_tool_circle_xz_harsh_gravity_harsh-on_contacts-on.csv --config configs\arm6_placeholder.yaml --json-out logs\control_benchmark_tool_circle_xz_harsh_metrics.json
```

Analyze existing benchmark CSV files without rerunning simulation:

```powershell
python tools\run_control_benchmarks.py --skip-run --scenario tool_circle_xz_harsh --scenario tool_square_xz_harsh --scenario tool_insert_line_harsh --ff gravity --harsh on --contacts on
```

## Render Log GIF

Render desired and actual `tool0` paths from an existing benchmark CSV. This is
offline and does not open MuJoCo:

```powershell
python tools\render_control_log_gif.py logs\control_benchmark_joint_circle_j2j3_harsh_gravity_harsh-on_contacts-on.csv --out logs\joint_circle_j2j3.gif --plane xz
python tools\render_control_log_gif.py logs\control_benchmark_joint_square_j2j3_harsh_gravity_harsh-on_contacts-on.csv --out logs\joint_square_j2j3.gif --plane xz
```

The GIF uses cyan for desired `tool0`, orange for actual `tool0`, and shows the
3D tool-position error over time on the right.

## Tune Parameters

Run the first-pass automatic tuner. It writes candidate configs and summaries
under `logs\tuning_runs\<timestamp>\`, but does not overwrite the canonical YAML
with the best result. With `tqdm` installed, candidate and scenario progress bars
are shown automatically. The tuner uses runtime parameter overrides, so it does
not regenerate files or rebuild the C executable; run `cmake --build build`
manually first if the benchmark binary is stale. Candidate 0 is the current
YAML baseline, candidate 1 is a model-derived seed, and later candidates are
internal CMA-style high-level scale samples around that seed. The tuner does
not depend on the external `cma` Python package. The default main loss is now
joint-space centric; task-space tool paths are check-only diagnostics.

```powershell
python tools\tune_control_params.py --budget 30 --seed 1
```

Smoke-test the tuner with a tiny budget and two quick scenarios:

```powershell
python tools\tune_control_params.py --budget 3 --seed 1 --scenario hold_zero_harsh --scenario step_j2_harsh
```

Force the random fallback instead of CMA-style evolution search:

```powershell
python tools\tune_control_params.py --budget 10 --seed 1 --optimizer random
```

Run one benchmark with a tuned override file:

```powershell
python tools\run_control_benchmarks.py --scenario step_j2_harsh --param-overrides logs\tuning_runs\<timestamp>\best.params --ff gravity --harsh on --contacts on
```

Treat old tuning runs as benchmark-specific. If benchmark trajectory generation
changes, rerun tuning instead of reusing an older `best.params`.

Inspect the model-derived seed and diagnostics from a tuning run:

```powershell
Get-Content logs\tuning_runs\<timestamp>\seed_diagnostics.json
Get-Content logs\tuning_runs\<timestamp>\summary.md
```
