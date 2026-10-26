#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from exercise3_task4.srv import Guard  

class GuardClient(Node):
    def __init__(self):
        # TODO: YOUR CODE HERE : 1 line
        # Initialize the node with the name 'guard_client' 
        super().__init__('guard_client')
        
        # Create a client for the Guard service
        self.client = self.create_client(Guard, '/guard_server')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /guard_server service...')

    def send_request(self, laps: int, dist: float):
        request = Guard.Request()
        request.laps = laps
        request.dist = dist
        self.get_logger().info(f'Sending guard request: laps={laps}, dist={dist}')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    guard_client = GuardClient()
    
    # 1. Guard once with distance of 0.6m and print the success log
    response1 = guard_client.send_request(laps=1, dist=0.6)
    guard_client.get_logger().info(f'First response: success={response1.success}')

    # TODO: YOUR CODE HERE : 2 lines
    # 2. Guard twice with distance of 0.4m and print the success log
    response2 = guard_client.send_request(laps=2, dist=0.4)
    guard_client.get_logger().info(f'Second response: success={response2.success}')

    guard_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
