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
        self._last_tf_warn_time = 0.0
        
        # Setup subscription
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        
        # Setup matplotlib plot (dynamic)
        self.gui_available = True
        try:
            import matplotlib.pyplot as plt
            plt.ion()  # interactive mode on
            self.fig, self.ax = plt.subplots(figsize=(10, 8))
            self.ax.scatter(self.map_points[:, 0], self.map_points[:, 1], c='black', s=20, marker='s', label='Static Map (Walls)')
            self.scan_scatter = self.ax.scatter([], [], c='red', s=8, alpha=0.6, label='Laser Scan (Transformed)')
            self.robot_scatter = self.ax.scatter([], [], c='blue', s=120, marker='o', zorder=5, label='Robot Pose')
            self.heading_arrow = None
            
            self.ax.set_title('Scan-to-Map ICP Live Alignment Verification')
            self.ax.set_xlabel('X (meters)')
            self.ax.set_ylabel('Y (meters)')
            self.ax.grid(True)
            self.ax.axis('equal')
            self.ax.legend()
            self.fig.canvas.draw()
            plt.pause(0.001)
        except Exception as e:
            self.gui_available = False
            self.get_logger().warn(f"No GUI display detected (or matplotlib error: {e}). Live plot disabled, printing stats continuously instead.")
            
        self.get_logger().info("Alignment checker initialized. Listening to /scan...")

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
        start_t = time.perf_counter()
        
        # Try to look up the transform from map to laser frame
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.25)
            )
        except Exception as e:
            curr_time = time.time()
            if curr_time - self._last_tf_warn_time > 3.0:
                self.get_logger().warn(f"Waiting for transform from 'map' to '{msg.header.frame_id}' (is scan_to_map_icp running?): {e}")
                self._last_tf_warn_time = curr_time
            return
        
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
        
        if len(ranges) == 0:
            return
        
        # Convert to 2D laser coordinates
        laser_pts = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
        
        # Transform to map frame
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)
        R = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
        transformed_pts = (R @ laser_pts.T).T + np.array([t.x, t.y])
        
        # Calculate nearest-neighbor distances for RMS error (scan points to map points)
        dists = np.linalg.norm(transformed_pts[:, np.newaxis, :] - self.map_points[np.newaxis, :, :], axis=2)
        min_indices = np.argmin(dists, axis=1)
        min_dists = dists[np.arange(len(transformed_pts)), min_indices]
        
        # Inliers threshold (same as C++ node, 0.4 meters)
        inlier_mask = min_dists < 0.4
        inliers = transformed_pts[inlier_mask]
        inlier_dists = min_dists[inlier_mask]
        
        rms = np.sqrt(np.mean(inlier_dists**2)) if len(inlier_dists) > 0 else 0.0
        inlier_ratio = len(inliers) / len(transformed_pts) if len(transformed_pts) > 0 else 0.0

        # Calculate map coverage ratio: what % of static map points are close to a scan point (within 15 cm)
        dists_map = np.linalg.norm(self.map_points[:, np.newaxis, :] - transformed_pts[np.newaxis, :, :], axis=2)
        min_dists_map = np.min(dists_map, axis=1)
        map_inliers_mask = min_dists_map < 0.15
        map_coverage_ratio = np.sum(map_inliers_mask) / len(self.map_points)
        
        # Update dynamic plot if GUI is available
        if self.gui_available:
            import matplotlib.pyplot as plt
            self.scan_scatter.set_offsets(transformed_pts)
            self.robot_scatter.set_offsets(np.array([[t.x, t.y]]))
            
            # Remove old heading arrow
            if self.heading_arrow:
                self.heading_arrow.remove()
                
            # Draw new heading arrow
            arrow_len = 0.3
            self.heading_arrow = self.ax.arrow(
                t.x, t.y,
                arrow_len * np.cos(yaw), arrow_len * np.sin(yaw),
                head_width=0.08, head_length=0.08, fc='blue', ec='blue', zorder=5
            )
            
            # Re-draw the plot dynamically
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            
        # Print results and profiling info
        end_t = time.perf_counter()
        elapsed_ms = (end_t - start_t) * 1000.0
        print(f"[Profiling] Python callback + plot update took: {elapsed_ms:6.2f} ms | Pose: [{t.x:7.3f}, {t.y:7.3f}, {yaw:6.3f} rad] | Map Coverage: {map_coverage_ratio*100.0:5.1f}% | Scan Inliers: {inlier_ratio*100.0:5.1f}% | RMS: {rms*100.0:5.2f} cm")

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
