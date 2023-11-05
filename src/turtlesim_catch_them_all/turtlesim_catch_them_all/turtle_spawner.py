#!/usr/bin/env python3
"""
This is a simple node that publishes a string message to the topic "my_first_topic"
"""  # noqa


from functools import partial
import math
from random import uniform
import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn, Kill
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.declare_parameter("spawn_frequency", 1)

        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value

        self.alive_turtles_ = []

        self.alive_turtles_publisher_ = self.create_publisher(
            TurtleArray, "alive_turtles", 10
        )

        self.catch_turtle_service_ = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle
        )

        self.spawn_turtle_timer_ = self.create_timer(
            1.0 / self.spawn_frequency_,  # type: ignore
            self.spawn_new_turtle,
        )

    def callback_catch_turtle(self, request, response):
        self.call_kill_server(request.name)
        response.success = True
        return response

    def call_kill_server(self, turtle_name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Turtlesim Server...")

        request = Kill.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill, turtle_name=turtle_name)
        )

    def callback_call_kill(self, future, turtle_name):
        try:
            future.result()
            for i, turtle in enumerate(self.alive_turtles_):
                if turtle.name == turtle_name:
                    del self.alive_turtles_[i]
                    self.publish_alive_turtles()
                    break
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    def spawn_new_turtle(self):
        TWO_PI = 2 * math.pi
        self.call_spawn_server(
            uniform(0.0, 11.0),
            uniform(0.0, 11.0),
            uniform(0.0, TWO_PI),
        )

    def call_spawn_server(self, x, y, theta):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Turtlesim Server...")

        request = Spawn.Request()

        request.x = x
        request.y = y
        request.theta = theta

        future = client.call_async(request=request)
        future.add_done_callback(
            partial(
                self.callback_call_spawn_server,
                x=x,
                y=y,
                theta=theta,
            )
        )

    def callback_call_spawn_server(self, future, x, y, theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(
                    f"Spawned {response.name} at x={x}, y={y}, theta={theta}"
                )

                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.alive_turtles_.append(new_turtle)

                self.publish_alive_turtles()

        except Exception as e:
            self.get_logger().error(
                f"Service call failed with unexpected error: {e}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)  # mantém o programa rodando
    rclpy.shutdown()


if __name__ == "__main__":
    main()
