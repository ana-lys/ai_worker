#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

possible_paths = [
    'src/ai_worker/ffw_collision_checker/explore/pareto_boundary_voxels.csv',
    'explore/pareto_boundary_voxels.csv',
    'pareto_boundary_voxels.csv',
    os.path.join(os.path.dirname(__file__), 'pareto_boundary_voxels.csv')
]

csv_file = None
for path in possible_paths:
    if os.path.exists(path):
        csv_file = path
        break

if not csv_file:
    print("Error: pareto_boundary_voxels.csv not found. Please run the C++ explorer first.")
    sys.exit(1)

print(f"Loading {csv_file}...")

import numpy as np
from scipy.spatial.transform import Rotation as R

# Generate the 125 target vectors the same way C++ did
ori_vals = [-np.pi/2, -np.pi/4, 0, np.pi/4, np.pi/2]
target_vecs = []
for r in ori_vals:
    for p in ori_vals:
        for yw in ori_vals:
            # C++ order: yaw * pitch * roll
            rot = R.from_euler('XYZ', [r, p, yw], degrees=False)
            # Apply to the X-axis to get the pointing vector
            vec = rot.apply([1.0, 0.0, 0.0])
            target_vecs.append(vec)

x_vals, y_vals, z_vals, v_scores = [], [], [], []
u_vals, v_dir_vals, w_vals, min_scores = [], [], [], []

with open(csv_file, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    for row in reader:
        if len(row) >= 129:
            x_vals.append(float(row[0]))
            y_vals.append(float(row[1]))
            z_vals.append(float(row[2]))
            v_scores.append(float(row[3]))
            
            # Scores for the 125 orientations are from index 4 to 128
            scores = [float(s) for s in row[4:129]]
            min_score = min(scores)
            min_idx = scores.index(min_score)
            
            min_scores.append(min_score)
            u_vals.append(target_vecs[min_idx][0])
            v_dir_vals.append(target_vecs[min_idx][1])
            w_vals.append(target_vecs[min_idx][2])

total_points = len(x_vals)

fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

import matplotlib.cm as cm
import matplotlib.colors as mcolors

print("Rendering 3D vectors for hardest directions...")
# Map v_scores directly to RGBA colors
norm = mcolors.Normalize(vmin=min(v_scores), vmax=max(v_scores))
cmap = plt.get_cmap('jet')
rgba_colors = cmap(norm(v_scores))

# Matplotlib 3D Quiver is notoriously bugged for coloring. 
# It creates 3N line segments in this exact bizarre order:
# [Shafts 1..N, Head1_L, Head1_R, Head2_L, Head2_R ... HeadN_L, HeadN_R]
# We must construct a 3N color array matching this exact order to prevent rainbow arrows.
correct_colors = np.concatenate([rgba_colors, np.repeat(rgba_colors, 2, axis=0)])

q = ax.quiver(x_vals, y_vals, z_vals, u_vals, v_dir_vals, w_vals, 
              length=0.06, normalize=True, arrow_length_ratio=0.3, alpha=0.8)
q.set_color(correct_colors)

# Create a manual ScalarMappable for the color bar
sm = cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])
cbar = fig.colorbar(sm, ax=ax, shrink=0.5, aspect=10, pad=0.05)
cbar.set_label('Average Versatility Score')

ax.set_xlabel('X (Forward/Back)')
ax.set_ylabel('Y (Left/Right)')
ax.set_zlabel('Z (Up/Down)')
ax.set_title(f'Right Arm Voxelized Pareto Boundary\n({total_points} voxels. Vector points to hardest orientation)')

# Set fixed limits based on the explore region
ax.set_xlim(0.3, 0.9)
ax.set_ylim(-0.75, 0.1)
ax.set_zlim(0.5, 2.1)

plt.show()
