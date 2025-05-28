#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts, AddTwoInts_Request, AddTwoInts_Response


class SimpleService(Node):
    def __init__(self):
        super().__init__('simple_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.handle_service_request_callback
        )

    def handle_service_request_callback(self, request: AddTwoInts_Request, response: AddTwoInts_Response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response
    
def main(args=None):
    rclpy.init(args=args)
    
    simple_service = SimpleService()
    rclpy.spin(simple_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()