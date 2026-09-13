from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_pipeline = LaunchConfiguration("start_pipeline")
    start_plan_adapter = LaunchConfiguration("start_plan_adapter")
    start_cartesian_approach = LaunchConfiguration("start_cartesian_approach")
    start_pick_pipeline = LaunchConfiguration("start_pick_pipeline")
    execute_trajectories = LaunchConfiguration("execute_trajectories")
    visualize_only_execution = LaunchConfiguration("visualize_only_execution")
    ompl_execution_backend = LaunchConfiguration("ompl_execution_backend")
    execute_gripper = LaunchConfiguration("execute_gripper")
    gripper_group_name = LaunchConfiguration("gripper_group_name")
    gripper_open_position = LaunchConfiguration("gripper_open_position")
    gripper_closed_position = LaunchConfiguration("gripper_closed_position")
    execution_start_tolerance = LaunchConfiguration("execution_start_tolerance")
    fail_on_start_state_mismatch = LaunchConfiguration("fail_on_start_state_mismatch")
    joint_state_wait_timeout_s = LaunchConfiguration("joint_state_wait_timeout_s")
    enable_cartesian_approach_recovery = LaunchConfiguration("enable_cartesian_approach_recovery")
    approach_recovery_ratio = LaunchConfiguration("approach_recovery_ratio")
    manage_target_collision = LaunchConfiguration("manage_target_collision")
    use_synthetic_pose = LaunchConfiguration("use_synthetic_pose")
    dry_run = LaunchConfiguration("dry_run")

    panda_demo_launch = PathJoinSubstitution(
        [
            FindPackageShare("moveit_resources_panda_moveit_config"),
            "launch",
            "demo.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_pipeline", default_value="true"),
            DeclareLaunchArgument("start_plan_adapter", default_value="false"),
            DeclareLaunchArgument("start_cartesian_approach", default_value="false"),
            DeclareLaunchArgument("start_pick_pipeline", default_value="false"),
            DeclareLaunchArgument("execute_trajectories", default_value="false"),
            DeclareLaunchArgument("visualize_only_execution", default_value="false"),
            DeclareLaunchArgument("ompl_execution_backend", default_value="execute_trajectory"),
            DeclareLaunchArgument("execute_gripper", default_value="false"),
            DeclareLaunchArgument("gripper_group_name", default_value="hand"),
            DeclareLaunchArgument("gripper_open_position", default_value="0.04"),
            DeclareLaunchArgument("gripper_closed_position", default_value="0.026"),
            DeclareLaunchArgument("execution_start_tolerance", default_value="0.05"),
            DeclareLaunchArgument("fail_on_start_state_mismatch", default_value="true"),
            DeclareLaunchArgument("joint_state_wait_timeout_s", default_value="5.0"),
            DeclareLaunchArgument("enable_cartesian_approach_recovery", default_value="true"),
            DeclareLaunchArgument("approach_recovery_ratio", default_value="0.5"),
            DeclareLaunchArgument("manage_target_collision", default_value="false"),
            DeclareLaunchArgument("use_synthetic_pose", default_value="true"),
            DeclareLaunchArgument("dry_run", default_value="true"),
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
                executable="aruco_pose_estimator",
                name="aruco_pose_estimator",
                output="screen",
                parameters=[{"use_synthetic_pose": use_synthetic_pose}],
                condition=IfCondition(start_pipeline),
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
                condition=IfCondition(start_pipeline),
            ),
            Node(
                package="panda_manipulation",
                executable="manipulation_marker_publisher",
                name="manipulation_marker_publisher",
                output="screen",
                condition=IfCondition(start_pipeline),
            ),
            Node(
                package="panda_manipulation",
                executable="target_object_manager",
                name="target_object_manager",
                output="screen",
                parameters=[{"manage_collision_object": manage_target_collision}],
                condition=IfCondition(start_pipeline),
            ),
            Node(
                package="panda_manipulation",
                executable="manipulation_task_manager",
                name="manipulation_task_manager",
                output="screen",
                parameters=[{"dry_run": dry_run}],
                condition=IfCondition(start_pipeline),
            ),
            Node(
                package="panda_manipulation",
                executable="moveit_plan_only_adapter",
                name="moveit_plan_only_adapter",
                output="screen",
                condition=IfCondition(start_plan_adapter),
            ),
            Node(
                package="panda_manipulation",
                executable="cartesian_approach_planner",
                name="cartesian_approach_planner",
                output="screen",
                condition=IfCondition(start_cartesian_approach),
            ),
            Node(
                package="panda_manipulation",
                executable="pick_plan_pipeline",
                name="pick_plan_pipeline",
                output="screen",
                parameters=[
                    {
                        "execute_trajectories": execute_trajectories,
                        "visualize_only_execution": visualize_only_execution,
                        "ompl_execution_backend": ompl_execution_backend,
                        "execute_gripper": execute_gripper,
                        "gripper_group_name": gripper_group_name,
                        "gripper_open_position": gripper_open_position,
                        "gripper_closed_position": gripper_closed_position,
                        "execution_start_tolerance": execution_start_tolerance,
                        "fail_on_start_state_mismatch": fail_on_start_state_mismatch,
                        "joint_state_wait_timeout_s": joint_state_wait_timeout_s,
                        "enable_cartesian_approach_recovery": enable_cartesian_approach_recovery,
                        "approach_recovery_ratio": approach_recovery_ratio,
                        "reset_to_home_before_execute": False,
                        "cartesian_min_fraction": 0.7,
                        "pre_grasp_orientation_tolerance": 1.2,
                        "planner_ids": [
                            "RRTConnectkConfigDefault",
                            "PRMkConfigDefault",
                            "RRTstarkConfigDefault",
                        ],
                    }
                ],
                condition=IfCondition(start_pick_pipeline),
            ),
        ]
    )
