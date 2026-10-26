#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10
        )
        self.subscription  # Prevents the unused-variable warning

    def callback(self, msg):
        front_dist = msg.ranges[0]
        left_dist = msg.ranges[90]
        right_dist = msg.ranges[270]
        
        self.get_logger().info(f"Fronte: {front_dist:.2f} m | Sinistra: {left_dist:.2f} m | Destra: {right_dist:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    lidar_reader = LidarReader()
    rclpy.spin(lidar_reader)
    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
