#!/usr/bin/env python3
import os
import sys
import math
import yaml
import numpy as np
import open3d as o3d
import cv2
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

def get_rosbag_options(bag_path):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    return storage_options, converter_options

def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options, converter_options = get_rosbag_options(bag_path)
    try:
        reader.open(storage_options, converter_options)
    except Exception:
        storage_options.storage_id = 'mcap'
        reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {tt.name: tt.type for tt in topic_types}

    scans = []
    odoms = []

    print("Reading bag file...")
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic not in ['/scan', '/ffw_laser_odom']:
            continue

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        if topic == '/scan':
            scans.append((t, msg))
        elif topic == '/ffw_laser_odom':
            odoms.append((t, msg))

    print(f"Loaded {len(scans)} scans and {len(odoms)} odoms.")
    return scans, odoms

def scan_to_points(scan_msg):
    angles = np.arange(len(scan_msg.ranges)) * scan_msg.angle_increment + scan_msg.angle_min
    ranges = np.array(scan_msg.ranges)
    valid = np.isfinite(ranges) & (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max) & (ranges > 0.01)
    ranges = ranges[valid]
    angles = angles[valid]

    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    z = np.zeros_like(x)
    return np.column_stack((x, y, z))

def get_tf_from_odom(odom_msg):
    p = odom_msg.pose.pose.position
    q = odom_msg.pose.pose.orientation

    x, y, z, w = q.x, q.y, q.z, q.w

    rot = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y),     0],
        [2*(x*y + w*z),     1 - 2*(x**2 + z**2), 2*(y*z - w*x),     0],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x**2 + y**2), 0],
        [0,                 0,                 0,                 1]
    ])

    trans = np.eye(4)
    trans[0, 3] = p.x
    trans[1, 3] = p.y
    trans[2, 3] = p.z

    return np.dot(trans, rot)

def fps(synced, num_points):
    if len(synced) <= num_points:
        return synced

    print(f"Performing farthest-point sampling to select {num_points} diverse frames...")
    poses = np.array([[odom.pose.pose.position.x, odom.pose.pose.position.y] for _, odom in synced])
    selected_indices = [0]
    distances = np.linalg.norm(poses - poses[0], axis=1)

    for _ in range(1, num_points):
        farthest = np.argmax(distances)
        selected_indices.append(farthest)
        new_distances = np.linalg.norm(poses - poses[farthest], axis=1)
        distances = np.minimum(distances, new_distances)

    return [synced[i] for i in selected_indices]

# ──────────────────────────────────────────────────────────────────────
#  WALL EXTRACTION – cluster-first + strict RANSAC + continuity check
# ──────────────────────────────────────────────────────────────────────

def _fit_line_ransac_2d(pts, distance_thresh=0.015, n_iterations=3000, min_inliers=15):
    """RANSAC line fit on Nx2 points. Returns (inlier_mask, direction, centroid) or None."""
    best_inliers = None
    best_count = 0
    n = len(pts)
    if n < min_inliers:
        return None

    for _ in range(n_iterations):
        i, j = np.random.choice(n, 2, replace=False)
        p1, p2 = pts[i], pts[j]
        d = p2 - p1
        seg_len = np.linalg.norm(d)
        if seg_len < 0.05:          # sample pair must be >= 5cm apart
            continue
        d /= seg_len
        normal = np.array([-d[1], d[0]])

        dists = np.abs((pts - p1) @ normal)
        inlier_mask = dists < distance_thresh
        count = np.count_nonzero(inlier_mask)

        if count > best_count:
            best_count = count
            best_inliers = inlier_mask

    if best_count < min_inliers:
        return None

    # Refine direction with PCA on inliers
    inlier_pts = pts[best_inliers]
    centroid = inlier_pts.mean(axis=0)
    cov = np.cov((inlier_pts - centroid).T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    direction = eigvecs[:, np.argmax(eigvals)]

    return best_inliers, direction, centroid


def _split_continuous_segments(pts, direction, max_gap=0.10):
    """Split inlier points into continuous segments (no gap > max_gap along the line).
    Returns list of sub-arrays of points."""
    centroid = pts.mean(axis=0)
    projections = (pts - centroid) @ direction
    order = np.argsort(projections)
    sorted_proj = projections[order]
    sorted_pts = pts[order]

    # Find gaps
    gaps = np.diff(sorted_proj)
    split_indices = np.where(gaps > max_gap)[0] + 1

    if len(split_indices) == 0:
        return [sorted_pts]

    segments = np.split(sorted_pts, split_indices)
    return segments


def _segment_to_wall(pts, direction):
    """From a continuous set of points and a direction, compute wall dict."""
    centroid = pts.mean(axis=0)
    projections = (pts - centroid) @ direction
    t_min, t_max = projections.min(), projections.max()
    start = centroid + t_min * direction
    end = centroid + t_max * direction
    length = t_max - t_min
    angle = math.degrees(math.atan2(direction[1], direction[0])) % 180.0
    density = len(pts) / max(length, 1e-6)  # points per meter
    return {
        'start': start,
        'end': end,
        'angle_deg': angle,
        'length': length,
        'n_inliers': len(pts),
        'density': density,
    }


def _angle_diff_mod90(a, b):
    """Smallest angular difference considering 0°/90°/180° equivalences."""
    diff = (a - b) % 180.0
    diff = min(diff, 180.0 - diff)   # fold to [0, 90]
    diff = min(diff, 90.0 - diff)    # fold to [0, 45] – distance to nearest 0 or 90
    return diff


def extract_walls(points_2d,
                  cluster_eps=0.06, cluster_min_samples=8,
                  ransac_dist=0.015, ransac_iters=3000,
                  min_wall_length=0.5, min_density=50.0,
                  max_gap=0.10, angle_tolerance_deg=5.0):
    """
    Strict wall extraction pipeline:
      1. DBSCAN cluster the 2D points so each cluster is a blob of nearby pts.
      2. For each cluster, RANSAC-fit a line. Split at gaps > max_gap.
      3. Keep only segments with length >= min_wall_length AND
         density >= min_density pts/m.
      4. Find the longest wall → reference yaw.
      5. Keep only walls within ±angle_tolerance_deg of 0° or 90° relative
         to the reference yaw.
    """
    print("\n[Wall Extraction] Clustering points (DBSCAN)...")
    db = DBSCAN(eps=cluster_eps, min_samples=cluster_min_samples).fit(points_2d)
    labels = db.labels_
    unique_labels = set(labels) - {-1}
    print(f"  Found {len(unique_labels)} clusters (noise pts: {np.sum(labels == -1)})")

    raw_walls = []

    for lab in unique_labels:
        cluster_pts = points_2d[labels == lab]
        if len(cluster_pts) < 10:
            continue

        # RANSAC within this cluster
        result = _fit_line_ransac_2d(cluster_pts, distance_thresh=ransac_dist,
                                      n_iterations=ransac_iters, min_inliers=10)
        if result is None:
            continue

        inlier_mask, direction, _ = result
        inlier_pts = cluster_pts[inlier_mask]

        # Split into continuous segments (no large gaps)
        segments = _split_continuous_segments(inlier_pts, direction, max_gap=max_gap)

        for seg_pts in segments:
            if len(seg_pts) < 5:
                continue
            wall = _segment_to_wall(seg_pts, direction)
            # Strict filters: length AND density
            if wall['length'] >= min_wall_length and wall['density'] >= min_density:
                raw_walls.append(wall)

    if not raw_walls:
        print("[Wall Extraction] No walls found after filtering.")
        return []

    # ── Step 4: identify longest wall → reference yaw ──
    raw_walls.sort(key=lambda w: w['length'], reverse=True)
    ref_wall = raw_walls[0]
    ref_angle = ref_wall['angle_deg']
    print(f"\n{'='*60}")
    print(f"[Wall Extraction] Reference wall (longest):")
    print(f"  Length  : {ref_wall['length']:.3f} m")
    print(f"  Angle   : {ref_angle:.2f}°")
    print(f"  Inliers : {ref_wall['n_inliers']}")
    print(f"  Density : {ref_wall['density']:.1f} pts/m")
    print(f"  Start   : ({ref_wall['start'][0]:.3f}, {ref_wall['start'][1]:.3f})")
    print(f"  End     : ({ref_wall['end'][0]:.3f}, {ref_wall['end'][1]:.3f})")
    print(f"{'='*60}")

    # ── Step 5: quantize – keep only 0°/90° relatives ──
    accepted = []
    ref_wall['is_reference'] = True
    accepted.append(ref_wall)

    for wall in raw_walls[1:]:
        diff = _angle_diff_mod90(wall['angle_deg'], ref_angle)
        if diff <= angle_tolerance_deg:
            wall['is_reference'] = False
            accepted.append(wall)

    # ── Print results ──
    print(f"\n[Wall Extraction] Accepted {len(accepted)} walls "
          f"(from {len(raw_walls)} candidates passing density/length):")
    print(f"  Min length   : {min_wall_length} m")
    print(f"  Min density  : {min_density} pts/m")
    print(f"  Max gap      : {max_gap} m")
    print(f"  Angle tol.   : ±{angle_tolerance_deg}°")
    print(f"  Reference yaw: {ref_angle:.2f}°\n")

    for i, w in enumerate(accepted):
        rel = _angle_diff_mod90(w['angle_deg'], ref_angle)
        family = "REF" if w.get('is_reference') else ("0°" if rel < 45 else "90°")
        print(f"  Wall {i:2d} | len={w['length']:.3f}m | "
              f"density={w['density']:.0f}pts/m | "
              f"angle={w['angle_deg']:.2f}° | Δ={rel:.2f}° | "
              f"family={family} | inliers={w['n_inliers']} | "
              f"({w['start'][0]:.3f},{w['start'][1]:.3f})->({w['end'][0]:.3f},{w['end'][1]:.3f})")

    print(f"{'='*60}\n")
    return accepted


def main():
    pkg_dir = os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_mapping')
    bag_path = None

    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    else:
        try:
            scan_dirs = [os.path.join(pkg_dir, d) for d in os.listdir(pkg_dir) if d.startswith('scan_') and os.path.isdir(os.path.join(pkg_dir, d))]
            if scan_dirs:
                bag_path = max(scan_dirs, key=os.path.getctime)
                print(f"Auto-selected most recent bag: {bag_path}")
        except Exception as e:
            pass

    if not bag_path or not os.path.exists(bag_path):
        print("Error: Could not find any valid bag path.")
        sys.exit(1)

    scans, odoms = read_bag(bag_path)
    if not scans or not odoms:
        sys.exit(1)

    print("Syncing scans with closest odometry...")
    synced = []
    for st, scan in scans:
        closest_odom = min(odoms, key=lambda x: abs(x[0] - st))
        synced.append((scan, closest_odom[1]))

    selected_frames = fps(synced, 20)

    print("Preprocessing pointclouds...")
    clouds = []
    for scan, odom in selected_frames:
        pts = scan_to_points(scan)
        if len(pts) < 10: continue

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)

        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        clouds.append({'pcd': pcd, 'odom': odom})

    print("Running Sequential ICP Merge...")
    merged_pcd = clouds[0]['pcd']
    base_odom_tf = get_tf_from_odom(clouds[0]['odom'])

    for i in range(1, len(clouds)):
        target = merged_pcd
        source = clouds[i]['pcd']
        odom_i_tf = get_tf_from_odom(clouds[i]['odom'])
        init_tf = np.linalg.inv(base_odom_tf) @ odom_i_tf

        result = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance=0.015,
            init=init_tf,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        if result.fitness < 0.1 or result.inlier_rmse > 0.015:
            source.transform(init_tf)
        else:
            source.transform(result.transformation)
        merged_pcd += source

    print("Cleaning final merged cloud...")
    # RELAXED filtering to keep edges but kill stray noise!
    merged_pcd, _ = merged_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    merged_pcd, _ = merged_pcd.remove_radius_outlier(nb_points=15, radius=0.05)
    
    map_out_path = os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_mapping/map.pcd')
    o3d.io.write_point_cloud(map_out_path, merged_pcd)
    print(f"Done! High-density map saved to {map_out_path}")

    # ── Wall extraction ──
    pts_2d = np.asarray(merged_pcd.points)[:, :2]
    walls = extract_walls(
        pts_2d,
        cluster_eps=0.06,          # DBSCAN: 6cm neighbor radius
        cluster_min_samples=8,     # DBSCAN: min 8 pts to form cluster
        ransac_dist=0.015,         # 1.5cm inlier threshold (strict)
        ransac_iters=3000,
        min_wall_length=0.5,       # discard segments < 0.5 m
        min_density=50.0,          # at least 50 pts per meter of wall
        max_gap=0.10,              # split at gaps > 10cm
        angle_tolerance_deg=5.0,   # ±5° from 0/90 relative to longest wall
    )

    # ── Draw wall map ──
    print("Generating Wall Map...")
    pts = np.asarray(merged_pcd.points)[:, :2]

    fig, ax = plt.subplots(figsize=(14, 14), dpi=150)
    # Point cloud in light gray
    ax.scatter(pts[:, 0], pts[:, 1], c='#cccccc', s=0.5, alpha=0.5, zorder=1)

    # Color scheme for wall families
    colors = {'REF': '#ff2222', '0°': '#00ccff', '90°': '#ff00ff'}

    for i, w in enumerate(walls):
        ref_angle = walls[0]['angle_deg'] if walls else 0
        rel = _angle_diff_mod90(w['angle_deg'], ref_angle)
        family = "REF" if w.get('is_reference') else ("0°" if rel < 45 else "90°")
        color = colors[family]

        sx, sy = w['start']
        ex, ey = w['end']
        # Draw thick wall line
        ax.plot([sx, ex], [sy, ey], color=color, linewidth=3.0, zorder=3,
                label=f"Wall {i} [{family}] {w['length']:.2f}m" if i < 15 else None)
        # Endpoint dots
        ax.plot([sx, ex], [sy, ey], 'o', color=color, markersize=6, zorder=4)
        # Label at midpoint
        mx, my = (sx + ex) / 2, (sy + ey) / 2
        ax.annotate(f"W{i} {w['length']:.2f}m\n{w['angle_deg']:.1f}°",
                    (mx, my), fontsize=7, fontweight='bold', color='white',
                    ha='center', va='center', zorder=5,
                    bbox=dict(boxstyle='round,pad=0.2', fc=color, alpha=0.85))

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title(f"Extracted Walls ({len(walls)} accepted)", fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    wall_img_path = os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_mapping/wall_map.png')
    fig.savefig(wall_img_path, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved Wall Map to {wall_img_path}")

    # Display using cv2 window
    img = cv2.imread(wall_img_path)
    if img is not None:
        if os.environ.get('DISPLAY'):
            print("Displaying in OpenCV window. Press any key to exit...")
            cv2.imshow("Wall Map", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("No DISPLAY found, skipping cv2.imshow.")

if __name__ == '__main__':
    main()
