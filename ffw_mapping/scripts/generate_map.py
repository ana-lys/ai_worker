#!/usr/bin/env python3
import os
import sys
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

    print("Generating Density Heatmap...")
    pts = np.asarray(merged_pcd.points)[:, :2]
    # Calculate density using KDTree
    pcd_tree = o3d.geometry.KDTreeFlann(merged_pcd)
    densities = []
    for i in range(len(merged_pcd.points)):
        [k, idx, _] = pcd_tree.search_radius_vector_3d(merged_pcd.points[i], 0.05)
        densities.append(k)
        
    densities = np.array(densities)
    
    plt.figure(figsize=(12, 12), dpi=150)
    plt.scatter(pts[:, 0], pts[:, 1], c=densities, cmap='viridis', s=1, norm=plt.matplotlib.colors.LogNorm())
    plt.colorbar(label='Density (points within 5cm radius)')
    plt.axis('equal')
    plt.grid(True)
    plt.title("Merged Point Cloud (Density/Intensity Map)")
    
    density_img_path = os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_mapping/density_map.png')
    plt.savefig(density_img_path, bbox_inches='tight')
    print(f"Saved Density Map to {density_img_path}")

    # Display using cv2 window as requested
    img = cv2.imread(density_img_path)
    if img is not None:
        if os.environ.get('DISPLAY'):
            print("Displaying in OpenCV window. Press any key to exit...")
            cv2.imshow("Density Map", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("No DISPLAY found, skipping cv2.imshow.")

if __name__ == '__main__':
    main()
