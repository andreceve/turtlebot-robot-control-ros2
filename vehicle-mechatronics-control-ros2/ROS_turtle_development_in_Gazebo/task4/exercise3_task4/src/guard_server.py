#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from exercise3_task4.srv import Guard  
import pandas as pd 
import os
import math
import time

# TODO: YOUR CODE HERE : 2 lines
# Import required messages 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class GuardServer(Node):
    def __init__(self):

        # Initialize dataframe for recording
        self.t0 = time.time()
        self.df = pd.DataFrame(columns=['x', 'y', 't'])

        # TODO: YOUR CODE HERE : ~ 5 lines
        # Initialize the node with the name 'guard_server' 
        super().__init__('guard_server')
        # Create publisher for the velocity command 
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Create the service server to use the "Guard" service type
        self.srv = self.create_service(Guard, '/guard_server', self.callback)
        # Create subscriber for the odometry topic (use "record" as callback)
        self.sub = self.create_subscription(Odometry, '/odom', self.record, 10)
        
        self.get_logger().info('Service /guard_server: Ready for operation')
    
    def record(self, msg):
        x = msg.twist.twist.linear.x
        y = msg.twist.twist.linear.y
        z = msg.twist.twist.angular.z
        th = 0.05  # Recording threshold
        
        if x > th or y > th or z > th:
            self.get_logger().info(f"x coordinate is {msg.pose.pose.position.x}")
            self.get_logger().info(f"y coordinate is {msg.pose.pose.position.y}")
            self.get_logger().info(f"time is {time.time() - self.t0}")

            self.df = pd.concat([self.df, pd.DataFrame([{
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                't': time.time() - self.t0
            }])], ignore_index=True)
                        
            if (self.df.shape[0] % 50) == 0:
                temp = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data.csv')
                print(f" ---------------------- {temp} ----------------------")
                self.df.to_csv(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data.csv'), index=False)

    # Helper function created to make easier the management of the movements
    def move_robot(self, linear_speed, angular_speed, duration):
        vel = Twist()
        vel.linear.x = linear_speed
        vel.angular.z = angular_speed
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            self.pub.publish(vel)
            time.sleep(0.01)
            
        # Stops the robot after the movement
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.pub.publish(vel)
        time.sleep(0.5) 

    def callback(self, request, response):
        self.get_logger().info("Service called")
        self.get_logger().info(f"Distance: {request.dist}")
        self.get_logger().info(f"Number of laps: {request.laps}")

        # TODO: YOUR CODE HERE : to execute the guarding task
        linear_speed = 0.2
        angular_speed = 0.5
        
        # Time calculation based on space and velocities
        time_forward = request.dist / linear_speed
        time_turn = math.pi / angular_speed  # math.pi is 180 degrees
        
        for lap in range(request.laps):
            self.get_logger().info(f"Esecuzione lap {lap + 1} di {request.laps}")
            # 1. Move forward
            self.move_robot(linear_speed, 0.0, time_forward)
            # 2. Turn 180 degrees
            self.move_robot(0.0, angular_speed, time_turn)
            # 3. Move forward
            self.move_robot(linear_speed, 0.0, time_forward)
            # 4. Turn 180 degrees
            self.move_robot(0.0, angular_speed, time_turn)
            # 5. Do nothing (already handled by the stop inside move_robot)
        
        response.success = True
        return response

# TODO: YOUR CODE HERE : 8 lines
# Define the main program and call it
def main(args=None):
    rclpy.init(args=args)
    guard_server = GuardServer()
    rclpy.spin(guard_server)
    guard_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
