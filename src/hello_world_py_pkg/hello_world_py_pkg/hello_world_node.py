#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    # All code goes between rclpy.init() and rclpy.shutdown()

    node = Node('new_test_node')
    node.get_logger().info('Hello, world!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()