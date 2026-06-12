# Pareto Boundary Extraction & Teleop Integration Memo

This document summarizes the complete workflow, from the initial exploration of the Pareto workspace to the final integration and refinement of the dual-arm teleoperation node.

## 1. Workspace Exploration & Pareto Generation
- **IK Workspace Explorer:** We developed `ffw_workspace_explorer.cpp` to systematically explore the kinematic feasibility of the arms.
- **125 Target Orientations:** We defined 125 unique target orientations. These represent a forward-facing 180-degree dome of coverage, with multiple roll ("Up" vector) variations for each forward direction.
- **Pareto Front:** By pushing the IK solver outward, we extracted the outermost physical boundary (the Pareto front) of the reachable workspace.

## 2. Voxelization & Boundary Post-Processing (`process_workspace.py`)
- **5cm Resolution:** We processed the raw point cloud data into a dense 5cm voxel grid (`VOXEL_SIZE = 0.05`). This provided a perfectly solid boundary layer consisting of **2,593 boundary voxels**.
- **The "Bleeding" Bug Fix:** Originally, a max-pooling strategy caused deep internal voxels (with perfect 1.0 versatility scores) to artificially inflate the scores of the outer border voxels. We fixed this by:
  - Calculating independent base scores for every voxel.
  - Averaging the scores of the 6 face-neighbors.
  - Applying a capped neighbor contribution (`smoothed_score = max(my_score, 0.25 * max_neighbor_score)`) so that perfectly capable neighbors couldn't artificially boost bad border scores. The average boundary score correctly dropped from a bugged 0.75 to a physically realistic 0.27.
- **Binary Export:** The processed boundaries were serialized into `pareto_boundary_voxels.bin` for rapid C++ ingestion.

## 3. Visualizing the Pareto Data
- **The "Globe with Flags":** We created `plot_125_vectors.py` to visualize the 125 orientations. It draws a translucent unit sphere, plotting the Forward (X) directions as red dots on the surface, and the Up (Z) vectors as blue flags sticking out of the dots.
- **Vector Field Visualization:** We created `plot_pareto_voxels.py` to plot the 3D boundary voxels. To visualize the "hardest direction to reach", we used 3D quiver arrows. We fixed a notorious Matplotlib 3D quiver coloring bug by mapping the versatility scores to RGBA and perfectly duplicating the color array 3 times. The arrows now point in the hardest direction and are solid-colored by their Average Versatility Score.

## 4. Teleop Integration (`ffw_ik_solver_teleop.cpp`)
- **Binary Ingestion:** The C++ teleop node now reads `pareto_boundary_voxels.bin` at startup, instantly populating the left and right voxel maps (`voxel_scores_l_` and `voxel_scores_r_`).
- **Global Pareto Bounds:** We calculate the absolute minimum and maximum X, Y, and Z voxel indices (`min_ix_`, `max_ix_`, etc.) directly from the loaded Pareto data to establish a global bounding box for the workspace.

## 5. Free Space Navigation & Decoupled Limits
- **Interior Free Space:** Initially, the teleop solver strictly required the arm to stay within explicitly mapped voxel coordinates ("soft bounds"). This inadvertently caused the arm to get stuck in the unmapped interior of the workspace. We removed this soft bound requirement. The arm is now constrained purely by the global Pareto bounding box ("hard bounds"), allowing it to move smoothly through unmapped interior "free space".
- **Decoupled Movement Constraints:** When the arm hit the boundaries or twisted past the `+- Pi/2` limits, the translation and rotation rejections were incorrectly coupled (e.g., twisting too far blocked all XYZ movement, causing a "stuck" feeling). We fully decoupled these:
  - Hitting the XYZ Pareto boundary blocks further translation but allows free rotation.
  - Exceeding the `+- Pi/2` rotation limit blocks further rotation but allows free XYZ translation.
- **Stable Orientation (Drift Fix):** We removed a faulty push-back mechanism that was subtracting rejected rotation deltas from the accumulated rotation state. This ensures the zero-point of the SpaceMouse orientation remains perfectly stable and prevents the massive orientation drift that previously occurred when hitting the rotation limits.
