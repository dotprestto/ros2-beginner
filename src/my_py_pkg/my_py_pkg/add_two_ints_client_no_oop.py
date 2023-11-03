#!/usr/bin/env python3
# flake8: noqa
"""
This is a simple node that publishes a string message to the topic "my_first_topic"
"""


import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_no_oop")

    client = node.create_client(AddTwoInts, "add_two_ints_no_oop")
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for Server Add Two Ints...")

    request = AddTwoInts.Request()
    request.a = 3
    request.b = 3
    future = client.call_async(request=request)

    rclpy.spin_until_future_complete(node=node, future=future)

    try:
        response = future.result()
        node.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
    except Exception as e:
        node.get_logger().error(f"Service call failed with unexpected error: {e}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
