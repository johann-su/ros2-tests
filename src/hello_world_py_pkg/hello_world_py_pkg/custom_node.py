#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_custom_node')
        self.counter_ = 0
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('My custom node has been started!')

    def timer_callback(self):
        self.get_logger().info(f'Time is ticking: {self.counter_}')
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)

    node = MyCustomNode()
    rclpy.spin(node) # Keep the node alive until shutdown is requested - also activates callbacks
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()