#!/usr/bin/env python3
"""
This is a simple node that publishes a string message to the topic "my_first_topic"
"""  # noqa


import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("node_name")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)  # mantém o programa rodando
    rclpy.shutdown()


if __name__ == "__main__":
    main()
