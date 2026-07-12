#!/usr/bin/env python3
import sys
import os
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

class AlignmentChecker(Node):
    def __init__(self, map_path):
        super().__init__('alignment_checker')
        self.map_path = map_path
        self.map_points = self.load_map(map_path)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        
        self.captured = False
        self.get_logger().info("Subscribed to /scan, waiting for transform from map...")

    def load_map(self, path):
        pts = []
        with open(path, 'r') as f:
            header_skipped = False
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.replace(',', ' ').split()
                if len(parts) != 3:
                    continue
                try:
                    # columns: wall_id x y
                    wall_id = float(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    pts.append([x, y])
                except ValueError:
                    if not header_skipped:
                        header_skipped = True
                        continue
        return np.array(pts)

    def scan_callback(self, msg):
        if self.captured:
            return
        
        # Try to look up the transform from map to laser frame
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                now,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except Exception as e:
            # Transform not available yet, wait for next callback
            return
        
        self.captured = True
        
        # Get transform translation and orientation
        t = transform.transform.translation
        q = transform.transform.rotation
        
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        # Parse laser scan points
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        # Filter valid ranges
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        
        # Convert to 2D laser coordinates
        laser_pts = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
        
        # Transform to map frame
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)
        R = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
        transformed_pts = (R @ laser_pts.T).T + np.array([t.x, t.y])
        
        # Calculate nearest-neighbor distances for RMS error
        dists = np.linalg.norm(transformed_pts[:, np.newaxis, :] - self.map_points[np.newaxis, :, :], axis=2)
        min_indices = np.argmin(dists, axis=1)
        min_dists = dists[np.arange(len(transformed_pts)), min_indices]
        
        # Inliers threshold (same as C++ node, 0.4 meters)
        inlier_mask = min_dists < 0.4
        inliers = transformed_pts[inlier_mask]
        inlier_dists = min_dists[inlier_mask]
        
        rms = np.sqrt(np.mean(inlier_dists**2)) if len(inlier_dists) > 0 else float('inf')
        inlier_ratio = len(inliers) / len(transformed_pts) if len(transformed_pts) > 0 else 0.0
        
        # Print results
        print("\n" + "="*60)
        print("ICP ALIGNMENT CHECKER RESULTS")
        print("="*60)
        print(f"Robot Position in Map Frame:")
        print(f"  X:     {t.x:8.4f} meters")
        print(f"  Y:     {t.y:8.4f} meters")
        print(f"  Yaw:   {yaw:8.4f} rad ({np.degrees(yaw):.2f} deg)")
        print(f"ICP Alignment Quality Metrics:")
        print(f"  Inlier count: {len(inliers)} / {len(transformed_pts)}")
        print(f"  Inlier ratio: {inlier_ratio*100:.1f}% (min threshold: 25%)")
        print(f"  Inlier RMS:   {rms:8.4f} meters ({rms*100:.2f} cm)")
        print("="*60)
        
        # Generate matplotlib plot
        self.plot_matplotlib(self.map_points, transformed_pts, [t.x, t.y], yaw)
        
        # Raise SystemExit to cleanly exit rclpy.spin
        sys.exit(0)

    def plot_matplotlib(self, map_pts, scan_pts, robot_pos, yaw):
        import matplotlib.pyplot as plt
        
        plt.figure(figsize=(10, 8))
        
        # Plot map points
        plt.scatter(map_pts[:, 0], map_pts[:, 1], c='black', s=20, marker='s', label='Static Map (Walls)')
        
        # Plot transformed scan points
        plt.scatter(scan_pts[:, 0], scan_pts[:, 1], c='red', s=8, alpha=0.6, label='Laser Scan (Transformed)')
        
        # Plot robot position and orientation arrow
        plt.scatter(robot_pos[0], robot_pos[1], c='blue', s=120, marker='o', zorder=5, label='Robot Pose')
        
        # Draw heading arrow
        arrow_len = 0.3
        plt.arrow(
            robot_pos[0], robot_pos[1],
            arrow_len * np.cos(yaw), arrow_len * np.sin(yaw),
            head_width=0.08, head_length=0.08, fc='blue', ec='blue', zorder=5
        )
        
        plt.title('Scan-to-Map ICP Alignment Verification')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        
        output_img = '/tmp/icp_alignment.png'
        plt.savefig(output_img, bbox_inches='tight')
        print(f"Saved plot image to {output_img}")
        
        # Try to show the plot if graphical display is available
        try:
            plt.show()
        except Exception:
            print("No GUI display detected. Plot saved as image instead.")

def main():
    map_path = '/home/lys/robotis_ws/src/ai_worker/ffw_mapping/all_walls_downsampled_rotated.txt'
    
    # Check if scan_to_map_icp is already running
    res = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
    node_already_running = '/scan_to_map_icp' in res.stdout
    
    proc = None
    if not node_already_running:
        print("Starting scan_to_map_icp node in the background...")
        proc = subprocess.Popen([
            'ros2', 'run', 'ffw_odom', 'scan_to_map_icp',
            '--ros-args',
            '-p', f'map_file:={map_path}',
            '-p', 'verbose:=false'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # Give the node a couple of seconds to start and converge
        time.sleep(3.0)
    else:
        print("Using existing running scan_to_map_icp node...")
        
    rclpy.init()
    checker = AlignmentChecker(map_path)
    try:
        rclpy.spin(checker)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if proc:
            print("Terminating background scan_to_map_icp node...")
            proc.terminate()
            proc.wait()
            print("Cleaned up background node.")

if __name__ == '__main__':
    main()
