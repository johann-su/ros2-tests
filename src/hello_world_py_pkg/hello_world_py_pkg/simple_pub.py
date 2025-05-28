#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class PublisherNode(Node):
    def __init__(self):
        super().__init__('simple_publisher_node')

        self.counter_ = 0

        self.publisher_ = self.create_publisher(String, 'my_cool_topic', 10) # 10 is the queue size
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'A new message appeared: {self.counter_}'
        self.publisher_.publish(msg)

        self.get_logger().info(f'Publishing: {msg.data}')

        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)

    node = PublisherNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()