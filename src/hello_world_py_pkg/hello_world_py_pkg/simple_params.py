#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class SimpleParams(Node):
    def __init__(self):
        super().__init__('simple_params')

        # Declare parameters with their types
        self.declare_parameter('my_str', rclpy.Parameter.Type.STRING)
        self.declare_parameter('my_int', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('my_double_array', rclpy.Parameter.Type.DOUBLE_ARRAY)

        # Get parameters
        param_str = self.get_parameter('my_str')
        param_int = self.get_parameter('my_int')
        param_double_array = self.get_parameter('my_double_array')
        
        self.get_logger().info("str: %s, int: %s, double[]: %s" %
                            (str(param_str.value),
                                str(param_int.value),
                                str(param_double_array.value),))

def main(args=None):
    rclpy.init(args=args)

    node = SimpleParams()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()