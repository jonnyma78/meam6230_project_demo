# RCM Modulation Demo

This repository contains a cleaned-up ROS 2 C++ structure for a laparoscope visual-servoing controller with an obstacle-aware dynamical-system modulation layer.

The code is split so that the project specific modulation logic is reproducible, while proprietary logic from the original system is represented as high-level placeholder logic.

## What is included

- `src/modulation.cpp`, `include/rcm_modulation_demo/modulation.hpp`  
  Reproducible cylindrical obstacle modulation logic.

- `src/rcm_tracking_node.cpp`, `include/rcm_modulation_demo/rcm_tracking_node.hpp`  
  ROS 2 node skeleton showing topic interfaces, TF lookup, control-loop flow, and where modulation is inserted.

- `src/controller_placeholders.cpp`, `include/rcm_modulation_demo/controller_placeholders.hpp`  
  Pseudocode and placeholder implementations for visual servoing and RCM maintenance logic that should not be reproduced directly.

- `config/params.yaml`  
  Example parameters for image intrinsics, RCM settings, and cylinder obstacle geometry.

## Important note

This is **not** a drop-in replacement for the original internal controller.

It is a clean teaching and review version intended to show:

1. the ROS 2 data flow,
2. how a nominal camera velocity is passed through a modulation layer,
3. how cylindrical obstacle geometry is used to locally reshape the velocity field,
4. where proprietary or non-public pieces would sit in the control pipeline.

## Build

Place this folder inside a ROS 2 workspace under `src/`, then build:

```bash
cd ~/ros2_ws
colcon build --packages-select rcm_modulation_demo
source install/setup.bash
```
