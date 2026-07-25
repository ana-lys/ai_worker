# ffw_joy Precision Mode Toggle

This memo documents the hardware-based Precision Mode toggle in the `ffw_joy` package.

## Motivation

The SpaceMouse physical hardware reaches its mechanical limits on diagonal movements before the internal axes hit 1.0 (e.g. `sqrt(0.6^2 + 0.6^2) = 0.84`). To achieve maximum velocity on diagonal movements, we use a `trans_sensitivity` multiplier of `1.5`.

Keeping this multiplier crushes the "fine movement" zone into the first 66% of the joystick's physical travel, making fine precision work difficult because the default cubic curve (`x^3`) combined with the 1.5x multiplier ramps up speed too aggressively for tiny movements.

To solve this, an independent Precision Mode scales down the velocity strictly when needed, without altering the underlying cubic curve shape.

---

## SpaceMouse (Dual-Device Mode)

### Hardware Trigger
Utilizes the two physical buttons on each SpaceMouse to create a toggle.

- **Trigger:** Double-click BOTH buttons (Button 0 and Button 1) simultaneously on a single SpaceMouse.
- **Why this works:**
  1. Operates independently per arm (toggling the left mouse only affects the left arm).
  2. Because Button 0 (Open Claw) and Button 1 (Close Claw) are pressed exactly simultaneously, the claw commands cancel out, preventing the claw from twitching or dropping an object.
  3. It does not conflict with the "Base/Arm" switch, which requires all four buttons across both mice to be double-clicked.

---

## Logitech G Extreme 3D Pro (Single-Device Mode)

### Hardware Trigger
Uses the dedicated trigger button (Button 0) on the Logitech joystick.

- **Trigger:** Each press of the trigger toggles precision mode for both arms simultaneously.
- **Why this works:**
  1. There is only one joystick, so per-arm toggling is not meaningful — the same device controls both arms.
  2. The trigger button has no other mapping and is easy to reach during teleoperation.
  3. A status message is published on `/precision_mode` topic.
- **Note:** The SpaceMouse dual-button double-click pattern is not used here because:
  1. There's only one device, so a "four-button combo" isn't possible.
  2. The Logitech buttons do not have claw cancel logic — simultaneous presses would not be safe.

---

## Mathematical Implementation (Both Devices)

When Precision Mode is toggled **ON**, the underlying math remains identical (cubic `x^3`), but the final velocity vector is multiplied by a **precision factor** (default `0.1`):

```cpp
Eigen::Vector3d trans_scaled = trans_raw * (trans_norm * trans_norm); // Cubic magnitude
if (precision_mode_) {
  trans_scaled *= 0.1;
}
```

This immediately grants exactly 10x finer control resolution for delicate tasks, while allowing the user to instantly switch back to full-speed coarse movement by toggling again.

## Base Teleop Precision Mode (Logitech) [sic]

For the base teleoperation node (`joy_base_teleop`), the Logitech trigger button also toggles a global precision multiplier applied to all twist velocity commands. When active, commanded velocities are scaled by the `precision_factor` parameter (default `0.3`), giving approximately 3x finer control for base movement and head/elevator positioning.
