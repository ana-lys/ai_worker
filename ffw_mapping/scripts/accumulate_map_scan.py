#!/usr/bin/env python3
"""
Accumulate all /map_scan PointCloud2 messages from a rosbag into a single PCD file.

Usage:
    ros2 run ffw_mapping accumulate_map_scan.py [bag_path]

If bag_path is omitted, processes the most recent scan_* bag in ffw_mapping.
"""

import os
import sys
import glob
import struct
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np


def get_rosbag_options(bag_path):
    return (
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
    )


def write_pcd_ascii(filename, points):
    """Write a PCD file in ASCII format."""
    n = len(points)
    with open(filename, 'w') as f:
        f.write("# .PCD v.7 - Point Cloud Data file format\n")
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def write_pcd_binary(filename, points):
    """Write a PCD file in binary format (more compact)."""
    n = len(points)
    with open(filename, 'wb') as f:
        header = (
            "# .PCD v.7 - Point Cloud Data file format\n"
            "VERSION .7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {n}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {n}\n"
            "DATA binary\n"
        )
        f.write(header.encode())
        for p in points:
            f.write(struct.pack('fff', float(p[0]), float(p[1]), float(p[2])))


def accumulate_map_scan(bag_path, output_binary=True):
    print(f"Reading bag: {bag_path}")

    reader = rosbag2_py.SequentialReader()
    storage_options, converter_options = get_rosbag_options(bag_path)

    try:
        reader.open(storage_options, converter_options)
    except Exception:
        storage_options.storage_id = 'mcap'
        reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {tt.name: tt.type for tt in topic_types}

    all_points = []
    msg_count = 0

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != '/map_scan':
            continue

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        msg_count += 1

        pts = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        all_points.extend(pts)

        if msg_count % 100 == 0:
            print(f"  Processed {msg_count} messages, {len(all_points)} points...")

    print(f"\nProcessed {msg_count} /map_scan messages")
    print(f"Raw accumulated points: {len(all_points)}")

    if not all_points:
        print("No points found — skipping PCD output.")
        return

    # Convert structured array to plain float64
    arr = np.array([(p[0], p[1], p[2]) for p in all_points], dtype=np.float64)

    # Simple grid filter: quantize to 2cm resolution, keep unique
    scale = 50.0  # 1/0.02
    rounded = np.round(arr[:, :2] * scale) / scale
    _, unique_idx = np.unique(rounded, axis=0, return_index=True)
    filtered = arr[unique_idx]
    print(f"After 2cm grid deduplication: {len(filtered)} points")

    # Determine output path
    bag_name = os.path.basename(bag_path.rstrip('/'))
    dataset_dir = os.path.dirname(bag_path.rstrip('/'))
    output_pcd = os.path.join(dataset_dir, f"{bag_name}_accumulated.pcd")

    if output_binary:
        write_pcd_binary(output_pcd, filtered)
    else:
        write_pcd_ascii(output_pcd, filtered)

    file_size = os.path.getsize(output_pcd)
    print(f"Saved: {output_pcd} ({file_size / 1024:.1f} KB)")
    return output_pcd


def main():
    rclpy.init()

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

    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
        if not os.path.exists(bag_path):
            print(f"Error: Path '{bag_path}' does not exist.")
            sys.exit(1)
        accumulate_map_scan(bag_path)
    else:
        scan_dirs = sorted(glob.glob(os.path.join(target_dir, "scan_*")))
        scan_dirs = [d for d in scan_dirs if os.path.isdir(d) and not d.endswith("_dataset")]
        if not scan_dirs:
            print("No scan_* bags found.")
            sys.exit(0)
        latest = scan_dirs[-1]
        print(f"Auto-selected latest bag: {latest}")
        accumulate_map_scan(latest)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
