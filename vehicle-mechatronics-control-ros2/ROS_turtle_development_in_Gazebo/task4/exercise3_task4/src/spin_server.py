#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time
import math 

# TODO 1: Import message for velocity publishing
from geometry_msgs.msg import Twist 

class SpinServer(Node):
    def __init__(self):
        # TODO 2: Initialize node and create publisher
        super().__init__('spin_server')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a service named '/spin_server' that uses the Empty service type
        self.service = self.create_service(Empty, '/spin_server', self.callback)
        self.get_logger().info('Service /spin_server ready for operation')

    def callback(self, request, response):
        self.get_logger().info('Service called. Rotating 360 degrees...')
        
        # TODO 3: Define message and set the velocity
        vel = Twist()
        vel.angular.z = 0.5  # Angular rotation speed (rad/s)
        
        # TODO 4: Use the "time" library to rotate 360 degrees and stop
        # Required time = 2*pi / angular speed
        duration = 2 * math.pi / vel.angular.z 
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            self.pub.publish(vel)
            time.sleep(0.1) # Short pause for the while loop
            
        # Stop the robot when the time is over
        vel.angular.z = 0.0
        self.pub.publish(vel)
        self.get_logger().info('Rotation complete.')

        return response

# TODO 5: Define the main program
def main(args=None):
    rclpy.init(args=args)
    spin_server = SpinServer()
    rclpy.spin(spin_server)
    spin_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
