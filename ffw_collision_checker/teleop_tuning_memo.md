# Teleop Tracking & Tuning Mechanics

This memo documents the mathematical interactions between the IK solver's P-controller and the teleoperation goal target.

## Why Do We Need a "Leash" (Error Margin)?
Our IK solver uses proportional control (Jacobian-based gradient descent) to calculate joint velocities. The fundamental equation is:
`Velocity = Error * step_size` 
*(where Error is the distance between the invisible target sphere and the physical robot hand).*

If we clamped the target directly to the physical hand every tick (0cm error margin), the error would always be zero. Even when the user pushes the joystick, the target would only move a microscopic fraction ahead of the arm for a single tick. The P-controller would generate a microscopic velocity, which gets further suppressed by the solver's internal `damping`. The result is a sluggish, unresponsive arm moving at a fraction of the commanded speed.

By allowing a **1.5cm leash** (error margin), the target is allowed to "stretch" ahead of the physical arm like a rubber band. The solver sees this 1.5cm error, generates a massive velocity vector, and fully overcomes the internal damping. The arm continuously chases the target at 100% of the joystick's commanded speed. 

If the arm hits a wall, the target is aggressively clamped to prevent it from wandering infinitely far away (preventing "spring wind-up"). When the user pulls back, the arm responds instantly.

## Tuning Guide

If the teleoperation feels sluggish, it is crucial to determine *why* it feels sluggish before changing parameters:

### Scenario A: Target Sphere Moves Slowly (but stays perfectly centered in the hand)
**Diagnosis:** The ROS `spnav_node` (SpaceMouse mapper) is scaling the joystick inputs too low. The target isn't moving fast enough, so the 1.5cm leash is never even reached.
**Fix:** Increase the `trans_scale_` or linear multiplier in the SpaceMouse mapper node to command a higher velocity.

### Scenario B: Target Sphere Hits the 1.5cm Limit and Slows Down
**Diagnosis:** The mapper is successfully sending the target far ahead, but the IK solver (`ffw_ik_solver_teleop.cpp`) is generating too little velocity to keep up, so the leash triggers and pulls the target back to limit its speed.
**Fix:**
1. **Increase P-Gain:** Increase `solver_cfg.step_size` (currently `0.15`) to e.g., `0.30`. This makes the arm move twice as fast to close the 1.5cm gap.
2. **Loosen Leash:** Increase the `max_dist` in `clip_target()` (e.g., from `0.015` to `0.030`). This allows the rubber band to stretch twice as far, generating twice the pulling force against the damping.
