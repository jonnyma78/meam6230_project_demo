# RCM Modulation Demo

This repository contains a cleaned-up ROS 2 C++ structure for a laparoscope visual-servoing controller with an obstacle-aware dynamical-system modulation layer.

The code is intentionally split so that the project-specific modulation logic is reproducible, while proprietary/non-public controller logic from the original Optosurgical system is represented only as high-level placeholder logic.

## What is included

- `src/modulation.cpp`, `include/rcm_modulation_demo/modulation.hpp`  
  Reproducible cylindrical obstacle modulation logic.

- `src/rcm_tracking_node.cpp`, `include/rcm_modulation_demo/rcm_tracking_node.hpp`  
  ROS 2 node skeleton showing topic interfaces, TF lookup, control-loop flow, and where modulation is inserted.

- `src/controller_placeholders.cpp`, `include/rcm_modulation_demo/controller_placeholders.hpp`  
  Pseudocode/placeholder implementations for visual servoing and RCM maintenance logic that should not be reproduced directly.

- `config/params.yaml`  
  Example parameters for image intrinsics, RCM settings, and cylinder obstacle geometry.

## Important note

This is **not** a drop-in replacement for the original internal controller. It is a clean teaching/review version intended to show:

1. the ROS 2 data flow,
2. how a nominal camera velocity is passed through a modulation layer,
3. how cylindrical obstacle geometry is used to locally reshape the velocity field,
4. where proprietary/non-public pieces would sit in the control pipeline.

## Build

Place this folder inside a ROS 2 workspace under `src/`, then build:

```bash
cd ~/ros2_ws
colcon build --packages-select rcm_modulation_demo
source install/setup.bash
```

## Run

```bash
ros2 run rcm_modulation_demo rcm_tracking_node --ros-args --params-file src/rcm_modulation_demo/config/params.yaml
```

The node publishes twist commands to:

```text
/servo_node/delta_twist_cmds
```

and expects a detected object center on:

```text
/object_center
```

where `point.x = u_px`, `point.y = v_px`, and `point.z = depth_m`.

## Suggested citation/description for report

The controller is structured as a nominal visual-servoing and RCM-constrained tracking pipeline followed by an obstacle-aware modulation layer. The modulation layer transforms the nominal camera velocity into the world frame, computes a local cylindrical obstacle basis, scales radial and tangential velocity components using a dynamical-system modulation matrix, and transforms the result back into the camera frame.
