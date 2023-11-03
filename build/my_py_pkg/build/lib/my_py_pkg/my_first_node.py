#!/usr/bin/env python3
"""
This is a simple node that publishes a string message to the topic "my_first_topic" 
"""  # noqa

import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node("py_test_node")
    node.get_logger().info("Hello ROS2!")
    rclpy.spin(node)  # mantém o programa rodando
    rclpy.shutdown()


if __name__ == "__main__":
    main()
