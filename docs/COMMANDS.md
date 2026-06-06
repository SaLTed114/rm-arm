# ArmSim Command Set

Run commands from the repository root.

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
python tools\run_control_benchmarks.py --scenario step_j2_harsh --scenario coupled_j2j3_harsh --scenario sine_j2_harsh --scenario straight_arm_lift_harsh --ff gravity --harsh on --contacts on
```

Run task-space tool path cases:

```powershell
python tools\run_control_benchmarks.py --scenario tool_circle_xz_harsh --scenario tool_square_xz_harsh --scenario tool_insert_line_harsh --ff gravity --harsh on --contacts on
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

For task-space benchmark GUI trails, cyan is the desired tool path and orange is
the actual `tool0` path.

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
