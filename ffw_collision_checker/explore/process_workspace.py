import csv
import math
import numpy as np
import sys
from collections import defaultdict

# 1. Target Orientations Generation (Must match C++)
ori_vals = [-math.pi/2, -math.pi/4, 0.0, math.pi/4, math.pi/2]
target_quats = []

def euler_to_quat(roll, pitch, yaw):
    # Match Eigen::AngleAxisd order: roll(X) * pitch(Y) * yaw(Z)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return np.array([qx, qy, qz, qw])

for r in ori_vals:
    for p in ori_vals:
        for y in ori_vals:
            target_quats.append(euler_to_quat(r, p, y))

VOXEL_SIZE = 0.10

print("Loading pareto_front_right.csv...")
# Map of voxel index (ix, iy, iz) -> list of reached quaternions
voxels = defaultdict(list)

try:
    with open('pareto_front_right.csv', 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            if len(row) >= 7:
                x, y, z = float(row[0]), float(row[1]), float(row[2])
                q = np.array([float(row[3]), float(row[4]), float(row[5]), float(row[6])])
                
                ix = int(math.floor(x / VOXEL_SIZE))
                iy = int(math.floor(y / VOXEL_SIZE))
                iz = int(math.floor(z / VOXEL_SIZE))
                voxels[(ix, iy, iz)].append(q)
except FileNotFoundError:
    print("pareto_front_right.csv not found! Make sure you run the C++ explorer first.")
    sys.exit(1)

print(f"Total populated voxels: {len(voxels)}")

print("Filtering boundary voxels...")
boundary_voxels = {}
directions = [
    (1,0,0), (-1,0,0),
    (0,1,0), (0,-1,0),
    (0,0,1), (0,0,-1)
]

for (ix, iy, iz), reached_qs in voxels.items():
    boundary_voxels[(ix, iy, iz)] = reached_qs

print(f"Total boundary voxels: {len(boundary_voxels)}")

print("Calculating Versatility Scores...")
# Save results
out_file = 'pareto_boundary_voxels.csv'
with open(out_file, 'w', newline='') as f:
    writer = csv.writer(f)
    header = ['x', 'y', 'z', 'versatility_score'] + [f'score_{i}' for i in range(125)]
    writer.writerow(header)
    
    for (ix, iy, iz), reached_qs in boundary_voxels.items():
        # Center of voxel for plotting
        vx = (ix + 0.5) * VOXEL_SIZE
        vy = (iy + 0.5) * VOXEL_SIZE
        vz = (iz + 0.5) * VOXEL_SIZE
        
        # Calculate max cosine score for each of the 125 target orientations
        scores = []
        for tq in target_quats:
            max_cos = -1.0
            for rq in reached_qs:
                # Dot product of quaternions
                dot = np.dot(tq, rq)
                # Cosine of angle difference
                cos_theta = 2.0 * (dot ** 2) - 1.0
                if cos_theta > max_cos:
                    max_cos = cos_theta
            scores.append(max_cos)
            
        versatility = sum(scores) / len(scores)
        
        row = [vx, vy, vz, versatility] + scores
        writer.writerow(row)

print(f"Done! Saved {len(boundary_voxels)} boundary voxels to {out_file}")
