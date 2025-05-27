#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('simple_subscriber_node')
        
        self.subscription_ = self.create_subscription(
            String,
            'my_cool_topic',
            self.listener_callback,
            10  # Queue size
        )

    def listener_callback(self, msg: String):
        self.get_logger().info(f'Received message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    node = SubscriberNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()