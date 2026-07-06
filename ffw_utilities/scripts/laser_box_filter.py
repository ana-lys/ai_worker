#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserBoxFilterNode(Node):
    def __init__(self):
        super().__init__('laser_box_filter_node')
        
        self.declare_parameter('min_x', -0.15)
        self.declare_parameter('max_x', 0.15)
        self.declare_parameter('min_y', -0.2)
        self.declare_parameter('max_y', 0.0)
        self.declare_parameter('input_topic', 'scan_raw')
        self.declare_parameter('output_topic', 'scan_filtered')
        
        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            rclpy.qos.qos_profile_sensor_data
        )
        
        self.get_logger().info(
            f'Laser box filter initialized: X[{self.min_x}, {self.max_x}], Y[{self.min_y}, {self.max_y}]'
        )

    def scan_callback(self, msg: LaserScan):
        filtered_msg = LaserScan()
        # Copy header and metadata
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        
        # Filter ranges
        ranges = list(msg.ranges)
        intensities = list(msg.intensities) if len(msg.intensities) > 0 else []
        has_intensities = len(intensities) > 0
        
        for i in range(len(ranges)):
            r = ranges[i]
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            # If point is inside the bounding box, cut it off
            if self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y:
                ranges[i] = 0.0
                if has_intensities:
                    intensities[i] = 0.0
                    
        filtered_msg.ranges = ranges
        filtered_msg.intensities = intensities
        
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserBoxFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
