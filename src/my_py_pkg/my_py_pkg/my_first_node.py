#!/usr/bin/env python3
"""
This is a simple node that publishes a string message to the topic "my_first_topic"
"""  # noqa


import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.get_logger().info("Hello ROS2 with OOP!")
        self.counter_ = 0
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info(f"Hello {self.counter_}")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)  # mantém o programa rodando
    rclpy.shutdown()


if __name__ == "__main__":
    main()
