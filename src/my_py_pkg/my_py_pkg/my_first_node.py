#!/usr/bin/env python3
"""
This is a simple node that publishes a string message to the topic "my_first_topic" 
"""  # noqa

from typing import List, Optional

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter


class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.get_logger().info("Hello ROS2 with OOP!")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)  # mantém o programa rodando
    rclpy.shutdown()


if __name__ == "__main__":
    main()
