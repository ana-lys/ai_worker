# XM430 Gripper — SMTM Variant

## Overview

The XM430-W350 Dynamixel gripper replaces the RH-P12-RN-A 4-bar linkage gripper on the
**right arm** of the FFW_SG2SMTM variant. It is a single revolute joint with a small
visual box representing the finger.

## Files

| File | Role |
|---|---|
| `ffw_description/urdf/common/xm430_gripper/xm430_gripper.urdf.xacro` | Macro definition |
| `ffw_description/urdf/ffw_sg2_smtm/ffw_sg2_smtm.urdf.xacro` | Caller (lines 152-154) |

## Macro Pose Chain

| Step | Element | Key values |
|---|---|---|
| **①** | `base_joint` (fixed) | `parent → base_link` |
| | | `xyz="${origin_xyz}"` = `0 0 -0.2` |
| | | `rpy="${origin_rpy}"` = `0 π π` |
| **②** | `base_link` | Red box `0.10×0.10×0.10` (motor housing) |
| **③** | `joint1` (revolute) | `base_link → finger_link`, origin at `0 0 0` |
| | | `axis="0 1 0"` (Y axis rotation) |
| | | Limits: `±2π` velocity/effort uncapped |
| **④** | `finger_link` | Green box `0.05×0.05×0.05` |
| | | Visual offset `0.08 0 -0.05` from joint origin |

## Changes Applied

### 1. Xacro block parameter workaround (xacro 2.1.1 bug)

- `*origin` block param replaced with `origin_xyz` + `origin_rpy` string params
- `<insert_block name="origin"/>` replaced with inline `<origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>`
- Prevents literal `<insert_block>` appearing in expanded output
- See `XACRO_BLOCK_PARAM_BUG.md` for details and list of affected files

### 2. Gripper X position (caller)

- Right arm (XM430): `origin_xyz="0.15 0 0"` → 15cm forward in X from arm_r_link7, no RPY flip
- Left arm (RH-P12-RN-A): `origin_xyz="0 0 -0.078"` → 7.8cm below arm_l_link7 (`rpy="0 π π"`)

### 3. Joint axis

- **Reverted** — attempted `0 -1 0` reverse, caused Gazebo physics clipping.
- Currently `axis="0 1 0"` — do not flip the URDF axis. If the motor direction
  is wrong, fix it in the controller parameter (sign inversion) instead.

## Launch

**Gazebo simulation:**
```bash
ros2 launch ffw_bringup ffw_sg2_smtm_follower_ai_gazebo.launch.py
```

**Teleop with SpaceMouse + IK solver (SMTM model):**
```bash
# Terminal 1 — Gazebo
ros2 launch ffw_bringup ffw_sg2_smtm_follower_ai_gazebo.launch.py

# Terminal 2 — SpaceMouse mappers (auto-detects devices)
ros2 launch ffw_collision_checker spacemouse_teleop.launch.py sim_only:=true

# Terminal 3 — IK solver with SMTM scene
# (Kill the bg2 solver from step 2 first: ros2 node kill /ffw_ik_solver_teleop)
ros2 run ffw_collision_checker ffw_ik_solver_teleop --ros-args \
  -p robot_model:=smtm -p hardware_mode:=false -p rate_hz:=50.0
```

## Shell Alias

Added to `~/.bashrc`:

- `ccbd` — walks up from any subdirectory to find workspace root,
  runs `colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`,
  sources `install/setup.bash`, returns to original directory.
- Plain `colcon build` prints a warning prompting use of `ccbd`.

## Known Issues

- `ffw_follower_body.xacro` and `ffw_leader_body.urdf.xacro` still use the
  broken `*origin` block parameter pattern (not critical unless body positioning changes).
- Left arm on SMTM still uses RH-P12-RN-A gripper (not XM430).
- XM430 finger direction reversal must be done in controller gains or mapping,
  not in the URDF joint axis.
