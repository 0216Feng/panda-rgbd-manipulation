from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="panda_manipulation",
                executable="planner_benchmark_runner",
                name="planner_benchmark_runner",
                output="screen",
            )
        ]
    )
