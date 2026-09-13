import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    marker_size_m = LaunchConfiguration("marker_size_m")
    object_center_offset_z_m = LaunchConfiguration(
        "object_center_offset_z_m"
    )
    required_samples = LaunchConfiguration("required_samples")
    package_share = get_package_share_directory("panda_manipulation")
    default_world = os.path.join(package_share, "worlds", "panda_table.sdf")
    gazebo_control_launch = os.path.join(
        package_share,
        "launch",
        "gazebo_control.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("world", default_value=default_world),
            # The rendered 45 mm plate calibrates to a 44 mm effective edge
            # with this Gazebo camera model and rasterization.
            DeclareLaunchArgument("marker_size_m", default_value="0.044"),
            DeclareLaunchArgument(
                "object_center_offset_z_m",
                default_value="-0.04",
            ),
            DeclareLaunchArgument("required_samples", default_value="10"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_control_launch),
                launch_arguments={
                    "gui": gui,
                    "world": world,
                    "move_to_ready": "false",
                    "run_gripper_test": "false",
                }.items(),
            ),
            Node(
                package="panda_manipulation",
                executable="aruco_pose_estimator",
                name="gazebo_aruco_pose_estimator",
                output="screen",
                parameters=[
                    {
                        "use_synthetic_pose": False,
                        "use_sim_time": True,
                        "base_frame": "panda_link0",
                        "camera_frame": "overhead_camera_optical_frame",
                        "marker_dictionary": "DICT_4X4_50",
                        "marker_id": 0,
                        "marker_size_m": marker_size_m,
                        "object_center_offset_z_m": object_center_offset_z_m,
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="aruco_perception_validator",
                name="gazebo_aruco_perception_validator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "required_samples": required_samples,
                    }
                ],
            ),
        ]
    )
