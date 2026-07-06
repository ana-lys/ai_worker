#!/usr/bin/env python3
import os
import sys
import yaml
import numpy as np
import open3d as o3d
import cv2
from sklearn.cluster import DBSCAN
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import tf_transformations

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
        # Fallback to mcap
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
    trans = tf_transformations.translation_matrix([p.x, p.y, p.z])
    rot = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
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

def ransac_line_fit(pts, threshold=0.03, iterations=500):
    best_inliers = np.zeros(len(pts), dtype=bool)
    if len(pts) < 2: return None, best_inliers
    
    for _ in range(iterations):
        idx = np.random.choice(len(pts), 2, replace=False)
        p1, p2 = pts[idx[0]], pts[idx[1]]
        v = p2 - p1
        v_len = np.linalg.norm(v)
        if v_len < 1e-5: continue
        v = v / v_len
        n = np.array([-v[1], v[0], 0])
        
        dists = np.abs(np.dot(pts - p1, n))
        inliers = dists < threshold
        if np.sum(inliers) > np.sum(best_inliers):
            best_inliers = inliers
            
    return best_inliers

def main():
    pkg_dir = os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_mapping')
    bag_path = None
    
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    else:
        # Find the most recent scan_* folder
        try:
            scan_dirs = [os.path.join(pkg_dir, d) for d in os.listdir(pkg_dir) if d.startswith('scan_') and os.path.isdir(os.path.join(pkg_dir, d))]
            if scan_dirs:
                bag_path = max(scan_dirs, key=os.path.getctime)
                print(f"Auto-selected most recent bag: {bag_path}")
        except Exception as e:
            pass
            
    if not bag_path or not os.path.exists(bag_path):
        print("Error: Could not find any valid bag path.")
        print("Please run the record_map.launch.py first to record the data.")
        sys.exit(1)

    scans, odoms = read_bag(bag_path)
    if not scans or not odoms:
        print("Error: No /scan or /ffw_laser_odom messages found in the bag.")
        sys.exit(1)

    print("Syncing scans with closest odometry...")
    synced = []
    for st, scan in scans:
        # Find closest odom
        closest_odom = min(odoms, key=lambda x: abs(x[0] - st))
        synced.append((scan, closest_odom[1]))

    selected_frames = fps(synced, 20)
    
    print("Preprocessing pointclouds...")
    clouds = []
    for scan, odom in selected_frames:
        pts = scan_to_points(scan)
        if len(pts) < 10:
            continue
            
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pcd = pcd.voxel_down_sample(voxel_size=0.02)
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        pcd.orient_normals_towards_camera_location(np.array([0., 0., 0.]))
        
        clouds.append({'pcd': pcd, 'odom': odom})

    if not clouds:
        print("Error: No valid pointclouds generated.")
        sys.exit(1)

    print("Running Sequential ICP Merge...")
    merged_pcd = clouds[0]['pcd']
    base_odom_tf = get_tf_from_odom(clouds[0]['odom'])

    for i in range(1, len(clouds)):
        target = merged_pcd
        source = clouds[i]['pcd']
        
        odom_i_tf = get_tf_from_odom(clouds[i]['odom'])
        init_tf = np.linalg.inv(base_odom_tf) @ odom_i_tf
        
        result = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance=0.1,
            init=init_tf,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        
        if result.fitness < 0.5 or result.inlier_rmse > 0.03:
            print(f"  -> Rejecting frame {i}: fitness={result.fitness:.2f}, rmse={result.inlier_rmse:.4f}")
            continue
            
        print(f"  -> Merged frame {i}: fitness={result.fitness:.2f}, rmse={result.inlier_rmse:.4f}")
        source.transform(result.transformation)
        merged_pcd += source

    print("Cleaning final merged cloud...")
    merged_pcd, _ = merged_pcd.remove_radius_outlier(nb_points=16, radius=0.05)
    
    print("Extracting Primitives...")
    merged_pts = np.asarray(merged_pcd.points)
    
    walls = []
    remaining = merged_pts.copy()
    N_WALLS = 15

    for _ in range(N_WALLS):
        if len(remaining) < 20: break
        inlier_mask = ransac_line_fit(remaining, threshold=0.04)
        if np.sum(inlier_mask) < 20: break
        
        inlier_pts = remaining[inlier_mask]
        
        centroid = np.mean(inlier_pts, axis=0)
        cov = np.cov(inlier_pts[:, :2].T)
        evals, evecs = np.linalg.eigh(cov)
        direction = evecs[:, 1]
        
        projections = np.dot(inlier_pts[:, :2] - centroid[:2], direction)
        p_min, p_max = np.min(projections), np.max(projections)
        
        p1 = centroid[:2] + direction * p_min
        p2 = centroid[:2] + direction * p_max
        
        walls.append({
            'x1': float(p1[0]), 'y1': float(p1[1]),
            'x2': float(p2[0]), 'y2': float(p2[1]),
            'thickness': 0.05
        })
        
        remaining = remaining[~inlier_mask]

    print(f"Extracted {len(walls)} walls.")

    # --- legs/poles: cluster leftover points, fit small circle ---
    print("Clustering leftover points for poles...")
    poles = []
    if len(remaining) > 0:
        labels = DBSCAN(eps=0.04, min_samples=4).fit_predict(remaining[:, :2])
        for label in set(labels) - {-1}:
            cluster = remaining[labels == label]
            if len(cluster) < 4: continue
            (cx, cy), r = cv2.minEnclosingCircle(cluster[:, :2].astype(np.float32))
            if r < 0.15:  # reject non-leg blobs
                poles.append({"cx": float(cx), "cy": float(cy), "r": float(r)})

    print(f"Extracted {len(poles)} poles.")

    map_out_path = os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_mapping/map.yaml')
    with open(map_out_path, 'w') as f:
        yaml.dump({"walls": walls, "poles": poles}, f)
        
    print(f"\nDone! Map saved to {map_out_path}")

if __name__ == '__main__':
    main()
