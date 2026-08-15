# SpaceMouse Precision Mode Toggle

This memo documents the addition of a hardware-based Precision Mode toggle in the `joy_hand` node (formerly `spacemouse_mapper`).

## Motivation
The SpaceMouse physical hardware reaches its mechanical limits on diagonal movements before the internal axes hit 1.0 (e.g. `sqrt(0.6^2 + 0.6^2) = 0.84`). To achieve maximum velocity on diagonal movements, we must use a `trans_sensitivity` multiplier of `1.5`. 

However, keeping this multiplier crushes the "fine movement" zone into the first 66% of the joystick's physical travel. This makes fine precision work extremely difficult because the default cubic curve (`x^3`) combined with the 1.5x multiplier ramps up speed too aggressively for tiny movements.

To solve this, we introduced an independent Precision Mode that scales down the velocity strictly when needed, without altering the underlying cubic curve shape.

## Hardware Trigger
We utilized the two physical buttons on each SpaceMouse to create a toggle.
- **Trigger:** Double-click BOTH buttons (Button 0 and Button 1) simultaneously on a single SpaceMouse.
- **Why this works:** 
  1. It operates independently per arm (toggling the left mouse only affects the left arm).
  2. Because Button 0 (Open Claw) and Button 1 (Close Claw) are pressed exactly simultaneously, the claw commands cancel out, preventing the claw from twitching or dropping an object.
  3. It does not conflict with the "Base/Arm" switch, which requires all four buttons across both mice to be double-clicked.

## Mathematical Implementation
When Precision Mode is toggled **ON**, the underlying math remains identical (Cubic `x^3`), but the final velocity vector is multiplied by **0.1**:

```cpp
Eigen::Vector3d trans_scaled = trans_raw * (trans_norm * trans_norm); // Cubic magnitude
if (precision_mode_) {
  trans_scaled *= 0.1;
}
```

This immediately grants exactly 10x finer control resolution for delicate tasks, while allowing the user to instantly switch back to full-speed coarse movement by double-clicking the buttons again.
