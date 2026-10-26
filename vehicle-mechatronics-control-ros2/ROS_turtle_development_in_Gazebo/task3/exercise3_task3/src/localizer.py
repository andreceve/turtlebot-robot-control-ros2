#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class Localizer(Node):
    def __init__(self):
        super().__init__('localizer')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.callback,
            10
        )
        self.subscription

    def euler_from_quaternion(self, quaternion_array):
        # Function to convert quaternion orientation to Euler angles (roll, pitch, yaw)
        x_e, y_e, z_e, w_e = quaternion_array
        
        sinr_cosp = 2 * (w_e * x_e + y_e * z_e)
        cosr_cosp = 1 - 2 * (x_e * x_e + y_e * y_e)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w_e * y_e - z_e * x_e)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        siny_cosp = 2 * (w_e * z_e + x_e * y_e)
        cosy_cosp = 1 - 2 * (y_e * y_e + z_e * z_e)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [pitch, roll, yaw]

    def callback(self, msg):
        # Extract position coordinates
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract orientation as a quaternion
        q = msg.pose.pose.orientation
        quat_array = [q.x, q.y, q.z, q.w]
        
        # Convert quaternion to Euler angles
        angles = self.euler_from_quaternion(quat_array)
        yaw_rad = angles[2] # Yaw is the third element returned by the function
        
        # Convert yaw from radians to degrees for the heading
        heading_deg = math.degrees(yaw_rad)
        
        # Print the final formatted output
        self.get_logger().info(f"Position: x={x:.2f}, y={y:.2f} | Heading: {heading_deg:.1f} degrees")

def main(args=None):
    rclpy.init(args=args)
    localizer = Localizer()
    rclpy.spin(localizer)
    localizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
