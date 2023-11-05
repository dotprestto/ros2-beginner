#!/usr/bin/env python3
"""
This is a simple node that publishes a string message to the topic "my_first_topic"
"""  # noqa


from functools import partial
import math

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.turtle_to_catch_ = None

        self.pose_ = None
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist,
            "turtle1/cmd_vel",
            10,
        )
        self.pose_subscriber_ = self.create_subscription(
            Pose,
            "turtle1/pose",
            self.callback_turtle_pose,
            10,
        )

        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray,
            "alive_turtles",
            self.callback_alive_turtles,
            10,
        )

        self.control_loop_timer_ = self.create_timer(
            1 / 100,
            self.control_loop,
        )

    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Turtlesim Server...")

        request = CatchTurtle.Request()

        request.name = turtle_name

        future = client.call_async(request=request)
        future.add_done_callback(
            partial(self.callback_call_catch_turtle, turtle_name)
        )

    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(
                    f"Turtle {turtle_name} could not be caught"
                )

        except Exception as e:
            self.get_logger().error(
                f"Service call failed with unexpected error: {e}"
            )

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            self.turtle_to_catch_ = msg.turtles[0]
            self.get_logger().error(f"FOLLOWING {self.turtle_to_catch_.name}")

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def control_loop(self):
        self.get_logger().error(f"AAAAAA {self.turtle_to_catch_}")
        if self.pose_ is None or self.turtle_to_catch_ is None:
            return

        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        msg = Twist()

        if distance > 0.5:
            # position
            msg.linear.x = 2 * distance

            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta

            # normalize angle
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            msg.angular.z = 6 * diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().error(f"REACHED {self.turtle_to_catch_.name}")

            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        self.cmd_vel_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)  # mantém o programa rodando
    rclpy.shutdown()


if __name__ == "__main__":
    main()
