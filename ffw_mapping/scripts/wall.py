#!/usr/bin/env python3

import os
import sys
import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from typing import List, Optional


def pca_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    centered = points - points.mean(axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    proj = centered @ vh[0]
    return proj.max() - proj.min()


def downsample_path(path: np.ndarray, target_spacing: float = 0.05) -> np.ndarray:
    if len(path) < 2:
        return path
    
    diffs = np.diff(path, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    cum_dist = np.concatenate(([0.], np.cumsum(segment_lengths)))
    total_length = cum_dist[-1]
    
    if total_length == 0:
        return path[[0, -1]] if len(path) > 1 else path
    
    target_dists = np.arange(0, total_length + target_spacing / 2, target_spacing)
    target_dists = np.clip(target_dists, 0, total_length)
    
    downsampled = [path[0].copy()]
    idx = 0
    for td in target_dists[1:]:
        while idx < len(cum_dist) - 1 and cum_dist[idx + 1] < td:
            idx += 1
        if idx >= len(cum_dist) - 1:
            break
        t = (td - cum_dist[idx]) / (cum_dist[idx + 1] - cum_dist[idx] + 1e-9)
        interp_point = path[idx] * (1 - t) + path[idx + 1] * t
        downsampled.append(interp_point)
    
    if len(downsampled) > 1:
        last = np.array(downsampled[-1])
        if np.linalg.norm(last - path[-1]) > target_spacing * 0.5:
            downsampled.append(path[-1].copy())
        else:
            downsampled[-1] = path[-1].copy()
    else:
        downsampled.append(path[-1].copy())
    
    return np.array(downsampled)


def fine_tune_path(cluster: np.ndarray, rough_path: np.ndarray, 
                   fine_window_radius: float = 0.025, fine_step_size: float = 0.01,
                   debug: bool = False) -> np.ndarray:
    if len(rough_path) < 2:
        return rough_path

    fine_tuned_nodes: List[np.ndarray] = []
    cluster = np.asarray(cluster)
    
    for i in range(len(rough_path) - 1):
        p_start = rough_path[i]
        p_end = rough_path[i + 1]
        segment_vector = p_end - p_start
        segment_length = np.linalg.norm(segment_vector)
        if segment_length < 1e-6:
            continue
            
        num_micro_steps = max(3, int(segment_length / fine_step_size) + 1)
        ts = np.linspace(0, 1, num_micro_steps)
        
        for t in ts:
            guidance_pos = p_start + t * segment_vector
            dists = np.linalg.norm(cluster - guidance_pos, axis=1)
            in_window_mask = dists <= fine_window_radius
            fine_window_pts = cluster[in_window_mask]
            n = len(fine_window_pts)
            
            if n >= 10:
                snapped_pos = fine_window_pts.mean(axis=0)
            elif n > 0:
                centroid = fine_window_pts.mean(axis=0)
                snapped_pos = (guidance_pos * 10.0 + n * centroid) / (10.0 + n)
            else:
                snapped_pos = guidance_pos.copy()
            
            fine_tuned_nodes.append(snapped_pos)

    return np.array(fine_tuned_nodes) if fine_tuned_nodes else np.array([])


def extract_line_segment_sliding_window(cluster: np.ndarray, base_window_radius: float = 0.06,
                                       base_step_size: float = 0.04, min_pts: int = 4,
                                       max_gap_steps: int = 5, debug: bool = False) -> np.ndarray:
    if len(cluster) < 10:
        return np.array([])

    remaining = cluster.copy()
    centroid = remaining.mean(axis=0)
    dists_to_cent = np.linalg.norm(remaining - centroid, axis=1)
    current_pos = remaining[np.argmax(dists_to_cent)].copy()
    
    tracked_cogs: List[np.ndarray] = []
    momentum_dir: Optional[np.ndarray] = None
    current_window_radius = base_window_radius
    gap_counter = 0
    last_safe_pos = current_pos.copy()
    last_safe_momentum: Optional[np.ndarray] = None
    
    max_iters = int(pca_length(cluster) / base_step_size * 3)
    
    iteration = 0
    while iteration < max_iters and len(remaining) > 0:
        dists = np.linalg.norm(remaining - current_pos, axis=1)
        in_window = dists <= current_window_radius
        consumed_pts = remaining[in_window]
        
        if len(consumed_pts) < 3 and tracked_cogs:
            current_pos = last_safe_pos.copy()
            if last_safe_momentum is not None:
                momentum_dir = last_safe_momentum.copy()
            current_window_radius = min(0.15, current_window_radius + 0.025)
            gap_counter += 1
            if gap_counter > max_gap_steps:
                break
            iteration += 1
            continue

        if len(consumed_pts) >= min_pts:
            local_cog = consumed_pts.mean(axis=0)
            remaining = remaining[~in_window]
            last_safe_pos = local_cog.copy()
            if momentum_dir is not None:
                last_safe_momentum = momentum_dir.copy()
            current_window_radius = base_window_radius
            gap_counter = 0
        else:
            local_cog = current_pos.copy()

        tracked_cogs.append(local_cog.copy())

        if momentum_dir is None:
            if len(consumed_pts) >= 3:
                local_pts = consumed_pts - local_cog
                _, _, vh = np.linalg.svd(local_pts, full_matrices=False)
                momentum_dir = vh[0]
                vec_to_cent = centroid - local_cog
                if np.dot(momentum_dir, vec_to_cent) < 0:
                    momentum_dir = -momentum_dir
            else:
                diff = centroid - local_cog
                norm = np.linalg.norm(diff)
                momentum_dir = diff / norm if norm > 1e-6 else np.array([1.0, 0.0])

        if len(remaining) == 0:
            break

        best_dir = momentum_dir.copy()
        max_density = -1
        test_angles = [0, np.pi/12, -np.pi/12, np.pi/6, -np.pi/6]
        
        for angle in test_angles:
            c, s = np.cos(angle), np.sin(angle)
            R = np.array([[c, -s], [s, c]])
            candidate_dir = R @ momentum_dir
            if np.dot(candidate_dir, momentum_dir) < -0.05:
                continue
            
            predicted_pos = local_cog + candidate_dir * base_step_size
            probe_dists = np.linalg.norm(remaining - predicted_pos, axis=1)
            density = np.sum(probe_dists <= current_window_radius)
            if angle == 0:
                density *= 1.1
            if density > max_density:
                max_density = density
                best_dir = candidate_dir

        step_dir = momentum_dir if (momentum_dir is not None and np.dot(best_dir, momentum_dir) < 0.85) else best_dir

        if max_density >= 8:
            current_step_size = base_step_size
        elif max_density <= 1:
            current_step_size = 0.0
            gap_counter += 1
            current_window_radius += 0.015
        else:
            current_step_size = (max_density / 8.0) * base_step_size
            gap_counter += 1
            current_window_radius += 0.01

        if gap_counter > max_gap_steps:
            break

        momentum_dir = best_dir
        current_pos = local_cog + step_dir * current_step_size
        iteration += 1

    return np.array(tracked_cogs) if tracked_cogs else np.array([])


def main():
    default_pcd = os.path.expanduser("~/robotis_ws/src/ai_worker/ffw_mapping/map_clean.pcd")
    pcd_path = sys.argv[1] if len(sys.argv) > 1 else default_pcd

    if not os.path.exists(pcd_path):
        print(f"File not found: {pcd_path}")
        return

    pcd_dir = os.path.dirname(pcd_path)

    print("Loading point cloud...")
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)[:, :2]

    cluster_eps = 0.06
    cluster_min_samples = 8
    min_wall_length = 0.8
    
    win_radius_rough = 0.06
    step_size_rough = 0.04
    min_pts_rough = 4
    max_gap_steps = 5
    
    win_radius_fine = 0.025
    step_size_fine = 0.01
    downsample_spacing = 0.05   # meters

    debug_rough = False
    debug_fine = False

    print("Clustering point cloud...")
    db = DBSCAN(eps=cluster_eps, min_samples=cluster_min_samples).fit(points)
    labels = db.labels_
    unique_labels = set(labels) - {-1}

    plt.figure(figsize=(12, 12))
    plt.scatter(points[:, 0], points[:, 1], s=1, c='lightgray', alpha=0.3, label='Raw Points')

    # Storage list for aggregating all points across all walls
    all_walls_data = []
    wall_paths = []  # Store fine paths for later analysis

    print("Extracting walls...")
    for label in sorted(unique_labels):
        cluster = points[labels == label]
        if len(cluster) < 15 or pca_length(cluster) < min_wall_length:
            continue

        rough_skeleton = extract_line_segment_sliding_window(
            cluster, win_radius_rough, step_size_rough, min_pts_rough, max_gap_steps, debug_rough
        )
        if len(rough_skeleton) < 2:
            continue

        fine_path = fine_tune_path(cluster, rough_skeleton, win_radius_fine, step_size_fine, debug_fine)
        if len(fine_path) < 2:
            continue

        wall_paths.append((label, fine_path))

        downsampled = downsample_path(fine_path, target_spacing=downsample_spacing)
        
        # Plot: smaller red dots
        plt.plot(downsampled[:, 0], downsampled[:, 1], '-', color='blue', linewidth=2.0, label=f'Wall {label} Line' if label == 0 else "")
        plt.scatter(downsampled[:, 0], downsampled[:, 1], s=15, c='red', zorder=5, label=f'Wall Points' if label == 0 else "")
        plt.scatter([downsampled[0, 0], downsampled[-1, 0]], 
                    [downsampled[0, 1], downsampled[-1, 1]], 
                    color='darkred', s=100, marker='o', zorder=6, edgecolors='yellow', linewidth=1.5)

        # Create an array [wall_id, x, y] for every point in this wall
        wall_id_col = np.full((len(downsampled), 1), label, dtype=np.float64)
        wall_segment_data = np.hstack((wall_id_col, downsampled))
        all_walls_data.append(wall_segment_data)
        
        print(f"Wall {label:3d} | Extracted {len(downsampled):3d} points")

    # === NEW: Find longest fine path and rotate to X-axis (no translation) ===
    if wall_paths:
        # Find longest fine path
        longest_label, longest_fine = max(wall_paths, key=lambda x: len(x[1]))
        print(f"Longest wall: {longest_label} with {len(longest_fine)} fine points")
        
        # Find longest straight segment before right angle turn
        def find_longest_straight_segment(path: np.ndarray) -> tuple:
            if len(path) < 3:
                return path, 0, len(path)
            
            max_length = 0
            best_start = 0
            best_end = len(path)
            
            for i in range(len(path) - 1):
                for j in range(i + 3, len(path)):
                    segment = path[i:j+1]
                    if len(segment) < 3:
                        continue
                    
                    # PCA to get direction
                    centered = segment - segment.mean(axis=0)
                    _, _, vh = np.linalg.svd(centered, full_matrices=False)
                    dir_vec = vh[0]
                    
                    # Check if segment is straight
                    total_len = np.sum(np.linalg.norm(np.diff(segment, axis=0), axis=1))
                    pca_len = pca_length(segment)
                    straightness = pca_len / (total_len + 1e-9)
                    
                    if straightness > 0.95:  # quite straight
                        seg_len = pca_len
                        if seg_len > max_length:
                            max_length = seg_len
                            best_start = i
                            best_end = j + 1
            
            if max_length > 0:
                return path[best_start:best_end], best_start, best_end
            return path, 0, len(path)
        
        straight_seg, start_idx, end_idx = find_longest_straight_segment(longest_fine)
        print(f"Found straight segment from idx {start_idx} to {end_idx}, length {pca_length(straight_seg):.3f}m")
        
        # Best fit line through straight segment → rotation only
        if len(straight_seg) >= 2:
            # Fit line: use PCA
            centered = straight_seg - straight_seg.mean(axis=0)
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
            principal_dir = vh[0]
            
            # Angle to rotate to X-axis
            angle = np.arctan2(principal_dir[1], principal_dir[0])
            cos_a, sin_a = np.cos(-angle), np.sin(-angle)
            
            # Rotation matrix only
            rotation_matrix = np.array([[cos_a, -sin_a],
                                       [sin_a, cos_a]])
            
            def transform_points(pts: np.ndarray) -> np.ndarray:
                if len(pts) == 0:
                    return pts
                # Rotation only around original origin (0,0)
                return pts @ rotation_matrix.T
            
            # Transform all fine paths
            print("Applying rotation to align longest straight wall to X-axis...")
            
            transformed_all_walls_data = []
            transformed_wall_paths = []
            
            for label, fine_path in wall_paths:
                trans_fine = transform_points(fine_path)
                transformed_wall_paths.append((label, trans_fine))
                
                trans_down = downsample_path(trans_fine, target_spacing=downsample_spacing)
                
                wall_id_col = np.full((len(trans_down), 1), label, dtype=np.float64)
                trans_data = np.hstack((wall_id_col, trans_down))
                transformed_all_walls_data.append(trans_data)
            
            # Use transformed data
            all_walls_data = transformed_all_walls_data
            wall_paths = transformed_wall_paths
            
            # Clear and replot
            plt.clf()
            plt.figure(figsize=(12, 12))
            
            print("Plotting rotated walls...")
            for label, fine_path in wall_paths:
                trans_down = downsample_path(fine_path, target_spacing=downsample_spacing)
                plt.plot(trans_down[:, 0], trans_down[:, 1], '-', color='blue', linewidth=2.0, 
                        label=f'Wall {label}' if label == longest_label else "")
                plt.scatter(trans_down[:, 0], trans_down[:, 1], s=15, c='red', zorder=5)
                plt.scatter([trans_down[0, 0], trans_down[-1, 0]], 
                            [trans_down[0, 1], trans_down[-1, 1]], 
                            color='darkred', s=100, marker='o', zorder=6, edgecolors='yellow', linewidth=1.5)
            
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title("Wall Centerlines - Rotated to X-axis (Original Origin)")
            plt.xlabel("X' (m)")
            plt.ylabel("Y' (m)")
            plt.grid(True, alpha=0.3)
            plt.legend()
            plt.tight_layout()
    
    # === SAVE ALL WALLS TO A SINGLE TEXT FILE ===
    if all_walls_data:
        combined_output = np.vstack(all_walls_data)
        txt_filename = os.path.join(pcd_dir, "all_walls_downsampled_rotated.txt")
        
        np.savetxt(txt_filename, combined_output, fmt='%d %.6f %.6f', delimiter=' ', 
                   header='wall_id x y', comments='')
        print(f"\nSuccess! Total points written: {len(combined_output)} → Saved: {txt_filename}")
    else:
        print("\nNo valid wall paths found to save.")

    plt.show()


if __name__ == "__main__":
    main()