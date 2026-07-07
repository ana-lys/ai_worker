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

def merge_lines(lines, angle_tol=np.pi/18, dist_tol=5.0): # dist_tol in pixels
    merged = []
    
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = np.arctan2(y2 - y1, x2 - x1)
        if angle < 0: angle += np.pi
        
        matched = False
        for m in merged:
            mx1, my1, mx2, my2 = m['coords']
            mangle = m['angle']
            
            diff = abs(angle - mangle)
            diff = min(diff, np.pi - diff)
            if diff > angle_tol:
                continue
                
            dx = mx2 - mx1
            dy = my2 - my1
            mag = np.hypot(dx, dy)
            if mag == 0: continue
            
            A = -dy / mag
            B = dx / mag
            C = -(A * mx1 + B * my1)
            
            dist1 = abs(A * x1 + B * y1 + C)
            dist2 = abs(A * x2 + B * y2 + C)
            
            if dist1 < dist_tol and dist2 < dist_tol:
                # Merge by finding extreme points along the line
                pts = np.array([[x1, y1], [x2, y2], [mx1, my1], [mx2, my2]])
                dir_vec = np.array([dx, dy]) / mag
                projs = np.dot(pts, dir_vec)
                
                min_idx = np.argmin(projs)
                max_idx = np.argmax(projs)
                
                m['coords'] = [pts[min_idx][0], pts[min_idx][1], pts[max_idx][0], pts[max_idx][1]]
                m['angle'] = np.arctan2(pts[max_idx][1] - pts[min_idx][1], pts[max_idx][0] - pts[min_idx][0])
                if m['angle'] < 0: m['angle'] += np.pi
                m['weight'] += 1
                matched = True
                break
                
        if not matched:
            merged.append({'coords': [x1, y1, x2, y2], 'angle': angle, 'weight': 1})
            
    return merged

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
    if os.environ.get('DISPLAY'):
        plt.show(block=False)

    print("Extracting robust lines via Hough Transform and Merging...")
    resolution = 0.02 # 2cm per pixel
    min_x, min_y = np.min(pts, axis=0)
    max_x, max_y = np.max(pts, axis=0)

    width = int((max_x - min_x) / resolution) + 1
    height = int((max_y - min_y) / resolution) + 1
    grid = np.zeros((height, width), dtype=np.uint8)

    px = ((pts[:, 0] - min_x) / resolution).astype(int)
    py = ((pts[:, 1] - min_y) / resolution).astype(int)
    grid[py, px] = 255

    grid_dilated = cv2.dilate(grid, np.ones((3,3), np.uint8), iterations=1)

    # Relaxed Hough transform so we detect MORE lines, then merge them!
    raw_lines = cv2.HoughLinesP(
        grid_dilated,
        rho=1,
        theta=np.pi/180,
        threshold=30,
        minLineLength=30,
        maxLineGap=10
    )

    # Black background, dark gray points
    img_bgr = np.zeros((height, width, 3), dtype=np.uint8)
    img_bgr[grid > 0] = (100, 100, 100) 

    if raw_lines is not None:
        print(f"Found {len(raw_lines)} raw line segments. Merging overlapping lines...")
        merged_lines = merge_lines(raw_lines, angle_tol=np.pi/18, dist_tol=10.0)

        # Sort by weight (confidence) so we can draw the most confident ones on top or thicker
        merged_lines.sort(key=lambda x: x['weight'], reverse=True)
        print(f"Reduced to {len(merged_lines)} compact, true edges!")

        for m in merged_lines:
            x1, y1, x2, y2 = m['coords']
            weight = m['weight']
            # Adjusted color scheme (BGR format)
            if weight > 5:
                color = (0, 255, 0) # Green for high confidence
                thickness = 3
            elif weight > 2:
                color = (255, 255, 0) # Cyan for medium confidence
                thickness = 2
            else:
                color = (0, 0, 255) # Red for low confidence
                thickness = 1

            cv2.line(img_bgr, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
    else:
        print("No lines found!")

    out_img_path = os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_mapping/extracted_walls.png')
    cv2.imwrite(out_img_path, img_bgr)
    print(f"Saved lines image to {out_img_path}")
    
    # Show it in a CV window as finish!
    if os.environ.get('DISPLAY'):
        print("Displaying in OpenCV window. Press any key to exit...")
        cv2.imshow("Extracted Walls", img_bgr)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No DISPLAY found, skipping cv2.imshow (image saved to disk).")

if __name__ == '__main__':
    main()
