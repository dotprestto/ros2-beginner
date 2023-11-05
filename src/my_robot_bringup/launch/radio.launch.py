from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot_names = ["Giskard", "BB8", "Daneel", "Jander", "C3PO"]
    robot_news_station_nodes = [
        Node(
            package="my_py_pkg",
            executable="robot_news_station",
            name=f"robot_news_station_{robot.lower()}",
            parameters=[{"robot_name": robot.lower()}],
        )
        for robot in robot_names
    ]

    smartphone = Node(
        package="my_py_pkg",
        executable="smartphone",
        name="smartphone",
    )

    for node in robot_news_station_nodes:
        ld.add_action(node)
    ld.add_action(smartphone)

    return ld
