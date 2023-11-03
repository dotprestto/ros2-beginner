#!/usr/bin/env python3
# flake8: noqa
"""
This is a simple node that publishes a string message to the topic "my_first_topic"
"""


import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.server_ = self.create_service(
            AddTwoInts, "add_two_ints", self.callback_add_two_ints
        )
        self.get_logger().info("Add two ints server has been started")

    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode()
    rclpy.spin(node)  # mantém o programa rodando
    rclpy.shutdown()


if __name__ == "__main__":
    main()
