from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot_news_publisher_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="my_news_pub",
        remappings=[("robot_news", "news")],
        parameters=[{"robot_name": "wall-e"}],
    )

    smartphone_node = Node(
        package="my_cpp_pkg",
        executable="smartphone",
        name="my_smartphone",
        remappings=[("robot_news", "news")],
    )

    ld.add_action(robot_news_publisher_node)
    ld.add_action(smartphone_node)

    return ld
