from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_scene_launch = PathJoinSubstitution(
        [
            FindPackageShare("panda_manipulation"),
            "launch",
            "moveit_scene.launch.py",
        ]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_scene_launch),
                launch_arguments={
                    "start_pipeline": "true",
                    "start_plan_adapter": "false",
                    "start_cartesian_approach": "false",
                    "start_pick_pipeline": "true",
                    "execute_trajectories": "false",
                    "visualize_only_execution": "true",
                    "manage_target_collision": "false",
                    "use_synthetic_pose": "true",
                    "dry_run": "true",
                }.items(),
            )
        ]
    )
