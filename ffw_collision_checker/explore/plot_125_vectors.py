#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

print("Generating the 125 target orientations on a Unit Sphere...")

ori_vals = [-np.pi/2, -np.pi/4, 0, np.pi/4, np.pi/2]

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# 1. Draw a faint, translucent Unit Sphere for context
u, v = np.mgrid[0:2*np.pi:40j, 0:np.pi:20j]
x_sphere = np.cos(u)*np.sin(v)
y_sphere = np.sin(u)*np.sin(v)
z_sphere = np.cos(v)
ax.plot_surface(x_sphere, y_sphere, z_sphere, color='gray', alpha=0.1, edgecolor='none')

x_fwd, y_fwd, z_fwd = [], [], []
u_up, v_up, w_up = [], [], []

for r in ori_vals:
    for p in ori_vals:
        for yw in ori_vals:
            rot = R.from_euler('XYZ', [r, p, yw], degrees=False)
            
            # Forward vector (X) puts the point exactly on the surface of the unit sphere
            vec_fwd = rot.apply([1.0, 0.0, 0.0])
            
            # Up vector (Z) acts as the "flag" showing how the orientation is twisted/rolled
            vec_up = rot.apply([0.0, 0.0, 1.0])
            
            x_fwd.append(vec_fwd[0]); y_fwd.append(vec_fwd[1]); z_fwd.append(vec_fwd[2])
            u_up.append(vec_up[0]); v_up.append(vec_up[1]); w_up.append(vec_up[2])

print("Rendering Unit Sphere projection...")

# Plot the Forward vectors as red dots on the sphere
ax.scatter(x_fwd, y_fwd, z_fwd, color='red', s=50, depthshade=True, label='Pointing Direction (Forward)')

# Plot the Up vectors as blue flags sticking out of the dots
ax.quiver(x_fwd, y_fwd, z_fwd, u_up, v_up, w_up, 
          length=0.25, normalize=True, color='blue', arrow_length_ratio=0.3, label='Twist / Roll (Up)')

# Draw origin axes for reference
ax.plot([-1.5, 1.5], [0, 0], [0, 0], color='black', alpha=0.3, linestyle='--')
ax.plot([0, 0], [-1.5, 1.5], [0, 0], color='black', alpha=0.3, linestyle='--')
ax.plot([0, 0], [0, 0], [-1.5, 1.5], color='black', alpha=0.3, linestyle='--')

ax.set_xlim([-1.2, 1.2])
ax.set_ylim([-1.2, 1.2])
ax.set_zlim([-1.2, 1.2])

ax.set_xlabel('X (Forward/Back)')
ax.set_ylabel('Y (Left/Right)')
ax.set_zlabel('Z (Up/Down)')
ax.set_title('The 125 Orientations mapped to a Unit Sphere\n(Red dot = Where it points, Blue arrow = How it rolls)')
ax.legend()

plt.show()
