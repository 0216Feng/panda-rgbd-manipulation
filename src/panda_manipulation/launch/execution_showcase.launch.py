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
                    "execute_trajectories": "true",
                    "visualize_only_execution": "false",
                    "ompl_execution_backend": "move_group",
                    "execute_gripper": "true",
                    "gripper_group_name": "hand",
                    "gripper_open_position": "0.04",
                    "gripper_closed_position": "0.026",
                    "fail_on_start_state_mismatch": "true",
                    "execution_start_tolerance": "0.05",
                    "joint_state_wait_timeout_s": "5.0",
                    "manage_target_collision": "false",
                    "use_synthetic_pose": "true",
                    "dry_run": "true",
                }.items(),
            )
        ]
    )
