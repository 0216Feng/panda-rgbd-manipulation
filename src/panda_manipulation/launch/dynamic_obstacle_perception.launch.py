import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    move_obstacle = LaunchConfiguration("move_obstacle")
    required_samples = LaunchConfiguration("required_samples")
    minimum_tracking_span_m = LaunchConfiguration("minimum_tracking_span_m")
    package_share = get_package_share_directory("panda_manipulation")
    control_launch = os.path.join(
        package_share,
        "launch",
        "gazebo_control.launch.py",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("move_obstacle", default_value="false"),
            DeclareLaunchArgument("required_samples", default_value="20"),
            DeclareLaunchArgument("minimum_tracking_span_m", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(control_launch),
                launch_arguments={
                    "gui": gui,
                    "move_to_ready": "false",
                }.items(),
            ),
            Node(
                package="panda_manipulation",
                executable="depth_obstacle_tracker",
                name="depth_obstacle_tracker",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="panda_manipulation",
                executable="dynamic_obstacle_perception_validator",
                name="dynamic_obstacle_perception_validator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "required_samples": required_samples,
                        "minimum_tracking_span_m": minimum_tracking_span_m,
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="dynamic_obstacle_controller",
                name="dynamic_obstacle_controller",
                output="screen",
                condition=IfCondition(move_obstacle),
                parameters=[
                    {
                        "use_sim_time": True,
                        "auto_start": True,
                    }
                ],
            ),
        ]
    )
