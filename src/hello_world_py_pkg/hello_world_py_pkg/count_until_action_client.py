#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from hello_world_interfaces.action import CountUntil


class CountUntilActionClientNode(Node):
    def __init__(self):
        super().__init__('count_until_action_client_node')
        self.get_logger().info('Count Until Action Client Node has been started.')

        self.action_client = ActionClient(self, CountUntil, 'count_until')

    def send_goal(self, count, period):
        # Wait for the server
        self.action_client.wait_for_server()

        # Create a goal
        goal = CountUntil.Goal()
        goal.count = count
        goal.period = period

        # Send the goal
        self.action_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle: ClientGoalHandle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info('Goal accepted, waiting for result...')
            self.goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.final_count}')

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilActionClientNode()
    node.send_goal(10, 1.0)  # Example goal: count to 5 with a period of 1 second
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()