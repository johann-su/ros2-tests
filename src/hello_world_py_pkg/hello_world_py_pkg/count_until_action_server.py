#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from hello_world_interfaces.action import CountUntil


class CountUntilActionServerNode(Node):
    def __init__(self):
        super().__init__('count_until_action_server_node')
        self.get_logger().info('Count Until Action Server Node has been started.')
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            self.execute_callback
        )

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Get the request from the goal handle
        self.get_logger().info(f'Received goal: {goal_handle.request.count}')
        target_num = goal_handle.request.count
        period = goal_handle.request.period

        # Exectute the goal
        self.get_logger().info('Starting counting...')
        counter = 0
        for i in range(target_num):
            counter += 1
            self.get_logger().info(f'Count: {counter}')
            time.sleep(period)

        # Once done, set the result...
        goal_handle.succeed()

        # ...and return the data
        result = CountUntil.Result()
        result.final_count = counter

        return result

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilActionServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()