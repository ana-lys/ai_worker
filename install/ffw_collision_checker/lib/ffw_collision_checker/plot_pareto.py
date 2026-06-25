import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys
import random

csv_file = 'pareto_front_right.csv'
if not os.path.exists(csv_file):
    print(f"Error: {csv_file} not found. Please run this script in the same directory where you executed the workspace explorer.")
    sys.exit(1)

print(f"Loading {csv_file}...")

x_vals, y_vals, z_vals = [], [], []
with open(csv_file, 'r') as f:
    reader = csv.reader(f)
    header = next(reader) # skip header
    for row in reader:
        if len(row) >= 3:
            x_vals.append(float(row[0]))
            y_vals.append(float(row[1]))
            z_vals.append(float(row[2]))

total_points = len(x_vals)

# Subsample if there are too many points for smooth matplotlib interaction
MAX_POINTS = 10000
if total_points > MAX_POINTS:
    print(f"Subsampling {total_points} points down to {MAX_POINTS} for smoother 3D interaction...")
    # Get random indices
    indices = random.sample(range(total_points), MAX_POINTS)
    x_vals = [x_vals[i] for i in indices]
    y_vals = [y_vals[i] for i in indices]
    z_vals = [z_vals[i] for i in indices]

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

print("Rendering 3D scatter plot...")
# Plot using Z coordinate for color mapping
scatter = ax.scatter(x_vals, y_vals, z_vals, 
                     s=5, c=z_vals, cmap='plasma', alpha=0.6)

# Add a color bar
cbar = fig.colorbar(scatter, ax=ax, shrink=0.5, aspect=10)
cbar.set_label('Z (Height)')

ax.set_xlabel('X (Forward/Back)')
ax.set_ylabel('Y (Left/Right)')
ax.set_zlabel('Z (Up/Down)')
ax.set_title(f'Right Arm Reachability Pareto Front\n({total_points} total points)')

# Optional: Ensure the axes scales are somewhat proportional
ax.set_xlim(0.3, 0.9)
ax.set_ylim(-0.75, 0.1)
ax.set_zlim(0.5, 2.1)

plt.show()
