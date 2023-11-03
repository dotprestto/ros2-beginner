#!/usr/bin/env python3
# flake8: noqa
"""
This is a simple node that publishes a string message to the topic "my_first_topic"
"""


import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")

        self.subscriber_ = self.create_subscription(
            String,
            "/robot_news",
            self.callback_robot_news,
            10,
        )
        self.get_logger().info("Smartphone has been started")

    def callback_robot_news(self, msg):
        self.get_logger().info(f"I received: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneNode()
    rclpy.spin(node)  # mantém o programa rodando
    rclpy.shutdown()


if __name__ == "__main__":
    main()
