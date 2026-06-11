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
