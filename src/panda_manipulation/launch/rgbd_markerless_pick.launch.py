import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    package_share = get_package_share_directory("panda_manipulation")
    pick_launch = os.path.join(package_share, "launch", "gazebo_pick.launch.py")
    rviz_config = os.path.join(
        package_share, "config", "rgbd_pointcloud_only.rviz"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pick_launch),
                launch_arguments={
                    "gui": gui,
                    "rviz": rviz,
                    "rviz_config": rviz_config,
                    "use_camera_perception": "false",
                    "use_rgbd_target_perception": "true",
                    "grasp_preclose_max_target_drift_m": "0.002",
                    # Calibrated for the conservative position servo: avoid
                    # palm contact while retaining finger-side contact.
                    "grasp_height_offset": "0.105",
                    "gripper_closed_position": "0.022",
                    "place_y": "-0.080",
                    "use_ompl_for_place": "true",
                    "use_cartesian_pre_place_transfer": "true",
                }.items(),
            ),
        ]
    )
