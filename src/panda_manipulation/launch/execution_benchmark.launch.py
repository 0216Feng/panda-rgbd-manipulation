from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    expected_trials = LaunchConfiguration("expected_trials")
    publish_period_s = LaunchConfiguration("publish_period_s")
    initial_delay_s = LaunchConfiguration("initial_delay_s")
    output_csv = LaunchConfiguration("output_csv")
    output_markdown = LaunchConfiguration("output_markdown")

    panda_demo_launch = PathJoinSubstitution(
        [
            FindPackageShare("moveit_resources_panda_moveit_config"),
            "launch",
            "demo.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("expected_trials", default_value="9"),
            DeclareLaunchArgument("publish_period_s", default_value="3.0"),
            DeclareLaunchArgument("initial_delay_s", default_value="4.0"),
            DeclareLaunchArgument("output_csv", default_value="pick_execution_benchmark.csv"),
            DeclareLaunchArgument("output_markdown", default_value="pick_execution_benchmark.md"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(panda_demo_launch)),
            Node(
                package="panda_manipulation",
                executable="scene_manager",
                name="scene_manager",
                output="screen",
                parameters=[{"publish_moveit_scene": True}],
            ),
            Node(
                package="panda_manipulation",
                executable="synthetic_target_sequence_publisher",
                name="synthetic_target_sequence_publisher",
                output="screen",
                parameters=[
                    {
                        "publish_period_s": publish_period_s,
                        "initial_delay_s": initial_delay_s,
                        "publish_after_result": True,
                        "repeat": False,
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="grasp_pose_generator",
                name="grasp_pose_generator",
                output="screen",
                parameters=[
                    {
                        "place_x": 0.45,
                        "place_y": -0.16,
                        "place_z": 0.26,
                        "grasp_height_offset": 0.14,
                        "lift_distance": 0.10,
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="manipulation_marker_publisher",
                name="manipulation_marker_publisher",
                output="screen",
            ),
            Node(
                package="panda_manipulation",
                executable="pick_plan_pipeline",
                name="pick_plan_pipeline",
                output="screen",
                parameters=[
                    {
                        "plan_once": False,
                        "execute_trajectories": True,
                        "visualize_only_execution": False,
                        "ompl_execution_backend": "move_group",
                        "execute_gripper": True,
                        "gripper_group_name": "hand",
                        "gripper_open_position": 0.04,
                        "gripper_closed_position": 0.026,
                        "fail_on_start_state_mismatch": True,
                        "execution_start_tolerance": 0.05,
                        "joint_state_wait_timeout_s": 5.0,
                        "cartesian_min_fraction": 0.7,
                        "enable_cartesian_approach_recovery": True,
                        "approach_recovery_ratio": 0.5,
                        "pre_grasp_orientation_tolerance": 1.2,
                        "planner_ids": [
                            "RRTConnectkConfigDefault",
                            "PRMkConfigDefault",
                            "RRTstarkConfigDefault",
                        ],
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="pick_plan_benchmark_recorder",
                name="pick_plan_benchmark_recorder",
                output="screen",
                parameters=[
                    {
                        "expected_trials": expected_trials,
                        "output_csv": output_csv,
                        "output_markdown": output_markdown,
                    }
                ],
            ),
        ]
    )
