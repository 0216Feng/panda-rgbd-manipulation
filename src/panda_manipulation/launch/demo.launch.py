from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_synthetic_pose = LaunchConfiguration("use_synthetic_pose")
    dry_run = LaunchConfiguration("dry_run")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_synthetic_pose", default_value="true"),
            DeclareLaunchArgument("dry_run", default_value="true"),
            Node(
                package="panda_manipulation_cpp",
                executable="trajectory_metrics_node",
                name="trajectory_metrics_node",
                output="screen",
            ),
            Node(
                package="panda_manipulation",
                executable="scene_manager",
                name="scene_manager",
                output="screen",
            ),
            Node(
                package="panda_manipulation",
                executable="aruco_pose_estimator",
                name="aruco_pose_estimator",
                output="screen",
                parameters=[{"use_synthetic_pose": use_synthetic_pose}],
            ),
            Node(
                package="panda_manipulation",
                executable="grasp_pose_generator",
                name="grasp_pose_generator",
                output="screen",
            ),
            Node(
                package="panda_manipulation",
                executable="manipulation_marker_publisher",
                name="manipulation_marker_publisher",
                output="screen",
            ),
            Node(
                package="panda_manipulation",
                executable="manipulation_task_manager",
                name="manipulation_task_manager",
                output="screen",
                parameters=[{"dry_run": dry_run}],
            ),
        ]
    )
