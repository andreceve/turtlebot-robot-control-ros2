#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class AutonomousDriver(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.linear_vel = 0.12 
        self.angular_vel = 0.0
        self.track_ended = False
        self.prev_error = 0.0

    def get_min_range(self, ranges, start_idx, end_idx):
        # Extracts the minimum distance in a specific LiDAR sector, ignoring 'inf' values
        slice_ranges = ranges[start_idx:end_idx]
        valid_ranges = [r for r in slice_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.0]
        if not valid_ranges:
            return 3.5
        return min(valid_ranges)

    def scan_callback(self, msg):
        if self.track_ended:
            return

        # ROBUST END-OF-TRACK DETECTION:
        # Instead of checking single rays which can slip through wall gaps,
        # we check the entire front 180 degrees (-90 to +90).
        # If the closest object in this entire huge sector is very far (> 3.0m) or inf,
        # it means we are in the open space at the end of the track.
        front_half = list(msg.ranges[0:90]) + list(msg.ranges[270:360])
        valid_front_half = [r for r in front_half if not math.isinf(r) and not math.isnan(r) and r > 0.0]
        
        if len(valid_front_half) == 0 or min(valid_front_half) > 3.0:
            self.get_logger().info("Track end detected (open space). Stopping the robot.")
            self.track_ended = True
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            return

        # Sector analysis for steering: find the closest obstacle on the left and right sides.
        # Using a wide angle (15 to 90 degrees) to be immune to zig-zag shapes.
        min_left = self.get_min_range(msg.ranges, 15, 90)
        min_right = self.get_min_range(msg.ranges, 270, 345)

        # Error calculation based on the closest protruding obstacles
        error = min_left - min_right

        # PD controller parameters
        Kp = 1.7  
        Kd = 0.9  # Slightly increased to damp the momentum before hitting the right wall
        
        derivative = error - self.prev_error
        
        # Calculate raw steering and cap it. Lowered max to 0.9 to prevent violent over-steering into walls.
        raw_angular = (Kp * error) + (Kd * derivative)
        self.angular_vel = max(-0.9, min(0.9, raw_angular))
        
        self.prev_error = error

        # Dynamic speed control based purely on frontal obstacles (-15 to +15 degrees).
        # We removed the lateral slow-down so the robot keeps momentum through the zig-zag.
        front_cone = list(msg.ranges[0:15]) + list(msg.ranges[345:360])
        valid_front = [r for r in front_cone if not math.isinf(r) and not math.isnan(r) and r > 0.0]
        min_front = min(valid_front) if valid_front else 3.5

        # Trigger the obstacle slow-down slightly earlier to give more room for the maneuver
        if min_front < 0.5:
            self.linear_vel = 0.06  # Slow down for the final pillar obstacle
        else:
            self.linear_vel = 0.12  # Normal cruising speed

    def timer_callback(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_vel
        vel_msg.angular.z = self.angular_vel
        self.publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
