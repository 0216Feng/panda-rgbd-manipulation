import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    package_share = get_package_share_directory("panda_manipulation")
    dynamic_launch = os.path.join(
        package_share,
        "launch",
        "dynamic_replanning_demo.launch.py",
    )
    rviz_config = os.path.join(
        package_share,
        "config",
        "rgbd_pointcloud_only.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(dynamic_launch),
                launch_arguments={
                    "gui": gui,
                    "rviz": "false",
                    "enable_motion_prediction": "true",
                    "prediction_horizon_s": "0.65",
                    "perception_required_samples": "4",
                    "perception_skip_initial_samples": "3",
                    "perception_minimum_span_m": "0.08",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rgbd_pointcloud_rviz",
                output="screen",
                arguments=["-d", rviz_config, "--fullscreen"],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
