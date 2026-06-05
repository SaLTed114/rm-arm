# ArmSim

ArmSim is split into a portable control core and a MuJoCo simulation harness.

## Layout

- `arm_core/`: pure C reusable arm core, including interfaces, reference sources, controllers, safety, and verification helpers. This is the part intended to be synced into a Keil/STM32F4xx project.
- `sim_mujoco/`: MuJoCo adapters, placeholder model, viewer, headless verifier, and CSV logging.
- `third_party/`: external PC-side dependencies such as MuJoCo and GLFW.
- `reference/`: read-only reference code. It is not built or modified by this project.

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
- `armsim_viewer`: MuJoCo viewer for visual joint direction checks.

The default model is `sim_mujoco/models/arm6_placeholder.xml`.

Keil/STM32 sync tooling will be added after the real-vehicle adapter exists.
