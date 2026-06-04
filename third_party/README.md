# Third Party Dependencies

This directory is for PC-side simulation dependencies only. Do not sync it into
the Keil/STM32 project.

If a dependency is itself a Git repository, add it as a submodule:

```powershell
git submodule add <repo-url> third_party/<name>
git submodule update --init --recursive
```

Do not vendor-copy Git repository contents directly into this directory. For
binary-only SDKs or manually downloaded releases, place them in the expected
layout below and document the source/version in this file or the project README.

Expected layout:

```text
third_party/
├─ mujoco/
│  ├─ include/mujoco/mujoco.h
│  └─ lib/ or bin/ containing the MuJoCo library
└─ glfw/
   └─ CMakeLists.txt or an installed glfw3 CMake package
```

The control core builds without anything in this directory.

Current local setup:

- `glfw/`: Git submodule at GLFW 3.4.
- `mujoco/`: ignored local MuJoCo 3.9.0 Windows x86_64 prebuilt SDK.
