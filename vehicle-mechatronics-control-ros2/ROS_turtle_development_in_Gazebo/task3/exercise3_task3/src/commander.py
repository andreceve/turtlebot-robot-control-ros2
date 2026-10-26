#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotCommander(Node):
    def __init__(self):
        super().__init__('commander')
        # Create a publisher on the '/cmd_vel' topic with a queue size of 10
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize the Twist message
        self.vel = Twist()
        
        # Set linear velocity (x axis, moving forward)
        self.vel.linear.x = 0.2
        
        # Set angular velocity (z axis, rotating)
        self.vel.angular.z = 0.5
        
        # Create a timer to publish the velocity command periodically (e.g., every 1.0 second)
        self.timer = self.create_timer(1.0, self.publish_velocity)

    def publish_velocity(self):
        # Publish the velocity message to the topic
        self.pub.publish(self.vel)
        self.get_logger().info('Publishing velocity command: linear x=0.2, angular z=0.5')

def main(args=None):
    rclpy.init(args=args)
    commander = RobotCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
