#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys
import math

possible_paths = [
    'src/ai_worker/ffw_collision_checker/explore/pareto_boundary_voxels.bin',
    'explore/pareto_boundary_voxels.bin',
    'pareto_boundary_voxels.bin',
    os.path.join(os.path.dirname(__file__), 'pareto_boundary_voxels.bin')
]

bin_file = None
for path in possible_paths:
    if os.path.exists(path):
        bin_file = path
        break

if not bin_file:
    print("Error: pareto_boundary_voxels.bin not found. Please run the C++ explorer first.")
    sys.exit(1)

print(f"Loading {bin_file}...")

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

x_blocked, y_blocked, z_blocked = [], [], []
interior_count = 0

# Fast binary loading
data = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 129)

for row in data:
    x = float(row[0])
    y = float(row[1])
    z = float(row[2])
    versatility = float(row[3])
    
    if versatility == 0.0:
        # Solid Block (Unreached / Capped)
        x_blocked.append(x)
        y_blocked.append(y)
        z_blocked.append(z)
    elif versatility < 1.0:
        # Soft Block (Pareto Boundary)
        x_vals.append(x)
        y_vals.append(y)
        z_vals.append(z)
        v_scores.append(versatility)
        
        # Scores for the 125 orientations are from index 4 to 128
        scores = row[4:129].tolist()
        min_score = min(scores)
        min_idx = scores.index(min_score)
        
        min_scores.append(min_score)
        u_vals.append(target_vecs[min_idx][0])
        v_dir_vals.append(target_vecs[min_idx][1])
        w_vals.append(target_vecs[min_idx][2])
    else:
        # Interior (No Block), versatility == 1.0
        interior_count += 1

VOXEL_SIZE = 0.05
# Create a set of soft block integer coordinates for fast neighbor lookup
soft_block_set = set()
for x, y, z in zip(x_vals, y_vals, z_vals):
    ix, iy, iz = int(math.floor(x / VOXEL_SIZE)), int(math.floor(y / VOXEL_SIZE)), int(math.floor(z / VOXEL_SIZE))
    soft_block_set.add((ix, iy, iz))

# Save the raw, unfiltered hard blocks to find true bounding box faces
x_blocked_all, y_blocked_all, z_blocked_all = list(x_blocked), list(y_blocked), list(z_blocked)

# Filter blocked voxels: only keep those adjacent to at least one soft block for the "ALL" view
filtered_x_blocked, filtered_y_blocked, filtered_z_blocked = [], [], []
directions_26 = []
for dx in [-1, 0, 1]:
    for dy in [-1, 0, 1]:
        for dz in [-1, 0, 1]:
            if dx == 0 and dy == 0 and dz == 0: continue
            directions_26.append((dx, dy, dz))

for x, y, z in zip(x_blocked, y_blocked, z_blocked):
    ix, iy, iz = int(math.floor(x / VOXEL_SIZE)), int(math.floor(y / VOXEL_SIZE)), int(math.floor(z / VOXEL_SIZE))
    is_adjacent = False
    
    # Check 26-neighborhood
    for dx, dy, dz in directions_26:
        if (ix + dx, iy + dy, iz + dz) in soft_block_set:
            is_adjacent = True
            break
            
    if is_adjacent:
        filtered_x_blocked.append(x)
        filtered_y_blocked.append(y)
        filtered_z_blocked.append(z)

x_blocked_adj, y_blocked_adj, z_blocked_adj = filtered_x_blocked, filtered_y_blocked, filtered_z_blocked

fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

import matplotlib.cm as cm
import matplotlib.colors as mcolors

# Plot Solid Blocks (Unreachable/Capped)
q_ref = [] # Will hold the quiver object

if len(x_blocked_all) > 0:
    print(f"Rendering {len(x_blocked_adj)} solid blocked voxels (adjacent to boundary)...")
    scatter_blocked = ax.scatter(x_blocked_adj, y_blocked_adj, z_blocked_adj, color='black', s=5, marker='o', alpha=0.5, label='Solid Block')
    
    bounds_map = {
        '1': ('X MIN (-X contour)', '-x'),
        '2': ('X MAX (+X contour)', '+x'),
        '3': ('Y MIN (-Y contour)', '-y'),
        '4': ('Y MAX (+Y contour)', '+y'),
        '5': ('Z MIN (-Z contour)', '-z'),
        '6': ('Z MAX (+Z contour)', '+z'),
    }

    def get_extremes(pts, dir_key):
        groups = {}
        for p in pts:
            x, y, z = p
            if dir_key in ['+x', '-x']:
                k = (round(y, 3), round(z, 3))
            elif dir_key in ['+y', '-y']:
                k = (round(x, 3), round(z, 3))
            else:
                k = (round(x, 3), round(y, 3))
                
            if k not in groups:
                groups[k] = []
            groups[k].append(p)
            
        extremes = []
        for k, group in groups.items():
            if dir_key == '+x': ext = max(group, key=lambda p: p[0])
            elif dir_key == '-x': ext = min(group, key=lambda p: p[0])
            elif dir_key == '+y': ext = max(group, key=lambda p: p[1])
            elif dir_key == '-y': ext = min(group, key=lambda p: p[1])
            elif dir_key == '+z': ext = max(group, key=lambda p: p[2])
            elif dir_key == '-z': ext = min(group, key=lambda p: p[2])
            extremes.append(ext)
        return extremes

    def on_key(event):
        if event.key == '0':
            scatter_blocked._offsets3d = (x_blocked_adj, y_blocked_adj, z_blocked_adj)
            if q_ref: q_ref[0].set_visible(True)
            fig.canvas.draw_idle()
            print(f"Displaying ALL adjacent hard blocks and soft blocks.")
        elif event.key in bounds_map:
            name, dir_key = bounds_map[event.key]
            # Get the single-layer shell points
            pts = list(zip(x_blocked_adj, y_blocked_adj, z_blocked_adj))
            extreme_pts = get_extremes(pts, dir_key)
            
            plot_x = [p[0] for p in extreme_pts]
            plot_y = [p[1] for p in extreme_pts]
            plot_z = [p[2] for p in extreme_pts]
            
            scatter_blocked._offsets3d = (plot_x, plot_y, plot_z)
            if q_ref: q_ref[0].set_visible(False)
            fig.canvas.draw_idle()
            print(f"Displaying {len(plot_x)} true contoured hard blocks for {name}. Soft blocks hidden.")
            
    fig.canvas.mpl_connect('key_press_event', on_key)
    print(">>> INTERACTIVE MODE: Press keys 1-6 to see the CONTOUR of the Hard Blocks in each direction (Soft blocks will hide). Press 0 to show Soft blocks + Adjacent Hard blocks. <<<")

# Plot Soft Blocks (Pareto Boundary)
if len(x_vals) > 0:
    print(f"Rendering {len(x_vals)} boundary vectors...")
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
                  length=0.06, normalize=True, arrow_length_ratio=0.3, alpha=0.8, label='Soft Block (Pareto)')
    q.set_color(correct_colors)
    q_ref.append(q)
    
    # Create a manual ScalarMappable for the color bar
    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, shrink=0.5, aspect=10, pad=0.05)
    cbar.set_label('Average Versatility Score')

ax.legend()

ax.set_xlabel('X (Forward/Back)')
ax.set_ylabel('Y (Left/Right)')
ax.set_zlabel('Z (Up/Down)')
ax.set_title(f'Right Arm Voxelized Pareto Workspace\n(Solid Blocks: {len(x_blocked_adj)}, Boundary: {len(x_vals)}, Interior: {interior_count})')

# Set fixed limits based on the explore region
ax.set_xlim(0.0, 0.9)
ax.set_ylim(-0.75, 0.1)
ax.set_zlim(0.5, 2.1)

plt.show()
