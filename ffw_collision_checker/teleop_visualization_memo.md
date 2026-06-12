# Teleop Visualization Improvements

This document summarizes the changes made to improve the MuJoCo visualization for the teleop task.

## What Was Done

1. **SpaceMouse Device Mapping Determinism**
   - Fixed an issue where the left and right SpaceMice were assigned non-deterministically via `/dev/input/jsX`.
   - Updated `spacemouse_teleop.launch.py` to identify the devices by their fixed physical USB hub paths (`/dev/input/by-path/`) and map them reliably to SDL indices 0 and 1.

2. **Mesh Transparency Tuning**
   - Modified the MuJoCo rendering loop in `ffw_ik_solver_teleop.cpp` to make the robot's physical meshes highly transparent, preventing them from blocking the view.
   - Core body parts (`head`, `arm_base_link`, `pole`, `drive`) were reduced to **2% opacity**.
   - Moving arm links were reduced to **15% opacity**.

3. **Orange Skeleton Overlay**
   - Implemented a custom `drawSkeleton` routine using MuJoCo's `mjv_connector`.
   - Drawn between every connected body and its parent in the kinematic chain.
   - Rendered as thick (1.5cm) capsules in a **vibrant orange** with **85% opacity**.
   - This "stick-figure" perfectly cuts through the transparent meshes to clearly show joint positions and alignments.

4. **Split-Screen Dual Camera View**
   - Split the `SimpleViewer` framebuffer vertically into two distinct viewports.
   - **Top Viewport**: A new camera (`cam_top_`) placed directly in front of the robot (Azimuth 180, Elevation -15) to monitor symmetrical alignment.
   - **Bottom Viewport**: Retained the original default camera, allowing for arbitrary user control while still having a fixed reference above.

5. **Decoupled Teleop Boundary Limits**
   - Fixed the "getting stuck" issue when the arm hits a boundary. Previously, the XYZ limits and Rotation limits were coupled, meaning if the orientation was twisted beyond its limit, all translation would be blocked as well.
   - Decoupled `trans_rejected` and `rot_rejected` so that:
     - Hitting an XYZ boundary blocks further translation but allows free rotation.
     - Exceeding the `+- Pi/2` rotation limit blocks further rotation but allows free translation.
   - Removed a faulty "push back" logic that was causing massive orientation drift on the right arm, ensuring the zero-point of the orientation remains stable and consistent.
   - Re-enabled the safe region logic to correctly update the teleop accumulators while driving into the safe region.

6. **Pareto Boundary Navigation (Free Space)**
   - Removed the strict requirement for the arm to only stay within explicitly mapped voxel points ("soft bounds").
   - XYZ translation is now constrained purely by the global Pareto bounding box ("hard bounds").
   - This allows the arm to move smoothly through unmapped interior "free space" without incorrectly getting blocked by missing voxel data.
