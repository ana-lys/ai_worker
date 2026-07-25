#!/usr/bin/env python3
import os
import sys
import math
from datetime import datetime
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.point_cloud2 import read_points

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

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

def convert_bag(bag_path):
    print(f"\nProcessing bag: {bag_path}")
    
    # Establish output dataset directory
    dataset_dir = bag_path.rstrip('/') + "_dataset"
    os.makedirs(dataset_dir, exist_ok=True)
    print(f"Output directory: {dataset_dir}")
    
    reader = rosbag2_py.SequentialReader()
    storage_options, converter_options = get_rosbag_options(bag_path)
    
    try:
        reader.open(storage_options, converter_options)
    except Exception:
        storage_options.storage_id = 'mcap'
        reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {tt.name: tt.type for tt in topic_types}

    # Open CSV files for writing
    scan_file = open(os.path.join(dataset_dir, "scan.csv"), "w")
    scan_file.write("timestamp_ns,angle_min,angle_max,angle_increment,time_increment,scan_time,range_min,range_max,ranges\n")

    odom_file = open(os.path.join(dataset_dir, "odom.csv"), "w")
    odom_file.write("timestamp_ns,x,y,yaw,vx,vy,wz\n")

    ekf_file = open(os.path.join(dataset_dir, "ekf_odom.csv"), "w")
    ekf_file.write("timestamp_ns,x,y,yaw,vx,vy,wz\n")

    icp_file = open(os.path.join(dataset_dir, "icp_pose_raw.csv"), "w")
    icp_file.write("timestamp_ns,x,y,yaw,vx,vy,wz\n")

    conf_file = open(os.path.join(dataset_dir, "confidence.csv"), "w")
    conf_file.write("timestamp_ns,confidence\n")

    map_scan_file = open(os.path.join(dataset_dir, "map_scan.csv"), "w")
    map_scan_file.write("timestamp_ns,num_points,points\n")

    counts = {"/scan": 0, "/odom": 0, "/ekf_odom": 0, "/icp_pose_raw": 0, "/scan_to_map_icp/confidence": 0, "/map_scan": 0}

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic not in counts:
            continue

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        counts[topic] += 1

        if topic == '/scan':
            ranges_str = " ".join(f"{r:.4f}" for r in msg.ranges)
            scan_file.write(f"{t},{msg.angle_min:.6f},{msg.angle_max:.6f},{msg.angle_increment:.6f},"
                            f"{msg.time_increment:.6f},{msg.scan_time:.6f},{msg.range_min:.4f},{msg.range_max:.4f},{ranges_str}\n")
        
        elif topic in ['/odom', '/ekf_odom', '/icp_pose_raw']:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = quaternion_to_yaw(msg.pose.pose.orientation)
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            wz = msg.twist.twist.angular.z
            
            line = f"{t},{x:.6f},{y:.6f},{yaw:.6f},{vx:.6f},{vy:.6f},{wz:.6f}\n"
            if topic == '/odom':
                odom_file.write(line)
            elif topic == '/ekf_odom':
                ekf_file.write(line)
            else:
                icp_file.write(line)

        elif topic == '/scan_to_map_icp/confidence':
            conf_file.write(f"{t},{msg.data:.6f}\n")

        elif topic == '/map_scan':
            # PointCloud2 — extract x,y for each point
            pts = []
            for p in read_points(msg, field_names=("x", "y"), skip_nans=True):
                pts.append(f"{p[0]:.4f},{p[1]:.4f}")
            points_str = ";".join(pts)
            map_scan_file.write(f"{t},{len(pts)},{points_str}\n")

    # Close all files
    scan_file.close()
    odom_file.close()
    ekf_file.close()
    icp_file.close()
    conf_file.close()
    map_scan_file.close()

    print("Conversion complete!")
    print("Messages processed:")
    for topic, count in counts.items():
        print(f"  {topic}: {count}")

def main():
    rclpy.init()
    
    # Locate mapping directories
    possible_dirs = [
        os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_mapping'),
        '/root/ros2_ws/src/ai_worker/ffw_mapping',
        '/root/robotis_ws/src/ai_worker/ffw_mapping',
    ]
    target_dir = possible_dirs[0]
    for d in possible_dirs:
        if os.path.exists(d):
            target_dir = d
            break

    # If bag path is provided as argument, convert that one
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
        if not os.path.exists(bag_path):
            print(f"Error: Path '{bag_path}' does not exist.")
            sys.exit(1)
        convert_bag(bag_path)
    else:
        # Auto-convert all bags in mapping directory matching scan_* (excluding datasets)
        print(f"Scanning directory: {target_dir} for scan_* bags...")
        entries = os.listdir(target_dir)
        scan_dirs = []
        for e in entries:
            full_path = os.path.join(target_dir, e)
            if os.path.isdir(full_path) and e.startswith("scan_") and not e.endswith("_dataset"):
                # verify it has mcap or db3 files
                has_files = any(f.endswith(".mcap") or f.endswith(".db3") for f in os.listdir(full_path))
                if has_files:
                    scan_dirs.append(full_path)

        if not scan_dirs:
            print("No scan_* bags found to convert.")
            sys.exit(0)

        for s_dir in sorted(scan_dirs):
            convert_bag(s_dir)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
