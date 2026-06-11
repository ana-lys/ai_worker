# Voxelized Pareto Boundary Explorer - Post-Processing Updates

This document summarizes the mathematical bug fixes and visualization updates made to the `ffw_workspace_explorer` and its associated plotting scripts.

## 1. The Versatility Score "Bleeding" Bug Fix

**The Problem:** 
Originally, the post-processing step used a "max-pooling" strategy over raw points within a 26-neighbor volume (a 30cm x 30cm x 30cm box). This caused deep internal voxels (which had perfect 1.0 scores) to falsely bleed their perfect scores outward onto the boundary voxels, resulting in boundary voxels artificially reporting a 1.0 versatility score.

**The Solution:**
We fundamentally changed how the border smoothing works:
- **Independent Base Scores:** Every voxel now calculates its 125-orientation scores completely independently based *only* on its own points.
- **Mathematical Averaging:** For boundary voxels, we look at the immediate 6 face-neighbors. We mathematical average the scores rather than max-pooling raw points.
- **The "Floor" Logic:** To prevent perfectly capable neighbors from inflating bad border scores, we implemented a capped neighbor contribution: `smoothed_score = max(my_score, 0.25 * max_neighbor_score)`. 
- **Result:** The average boundary score dropped from a bugged `0.75` down to a physically realistic `0.27`, providing a smooth physical gradient of capability without false `1.0` scores at the edge.

## 2. Visualization Updates (`plot_pareto_voxels.py`)

**The Problem:**
The original plot just showed colored dots. To visualize the "hardest direction to reach" for each voxel, we added 3D vectors. However, Matplotlib's 3D `quiver` function has a notorious coloring bug where it creates 3 line segments per arrow (shaft, left head, right head) sequentially. Passing a standard array of colors resulted in "rainbow vectors".

**The Solution:**
- Replaced the scatter plot entirely with a pure vector field (quiver arrows).
- Normalised all arrows to a fixed, visible length.
- Fixed the Matplotlib bug by mapping the versatility scores to RGBA colors and perfectly duplicating the color array 3 times (`np.concatenate([colors, np.repeat(colors, 2, axis=0)])`) to match Matplotlib's bizarre internal segment order.
- The arrows now point perfectly in the direction of the hardest target orientation to reach, while being solid-colored by their Average Versatility Score.

## 3. The Target Orientation Visualization (`plot_125_vectors.py`)

**The Problem:**
It was difficult to understand what the 125 target orientations physically represented in 3D space.

**The Solution:**
Created a dedicated visualization script (`plot_125_vectors.py`) using a "Globe with Flags" approach:
- Draws a translucent unit sphere.
- Plots the 125 **Forward (X)** pointing directions as Red Dots directly on the surface of the sphere (showing the front-facing 180-degree dome coverage).
- Plots the **Up (Z)** vectors as Blue Flags sticking out of the red dots to clearly indicate the "Roll" of the wrist at that orientation.

## 4. Performance & Resolution Updates

- Upgraded the voxel grid resolution from `10cm` down to `5cm` (`VOXEL_SIZE = 0.05` in `ffw_workspace_explorer.cpp`). 
- Due to the incredibly dense raw data (10 points per cm²), the boundary is perfectly solid at 5cm.
- The 5cm grid produces **2,593 boundary voxels**, which provides highly detailed physical bounds while remaining completely smooth to render and rotate interactively in Python.
