import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    planner_id = LaunchConfiguration("planner_id")
    obstacle_start_y = LaunchConfiguration("obstacle_start_y")
    obstacle_end_y = LaunchConfiguration("obstacle_end_y")
    obstacle_speed_mps = LaunchConfiguration("obstacle_speed_mps")
    obstacle_trigger_delay_s = LaunchConfiguration("obstacle_trigger_delay_s")
    perception_required_samples = LaunchConfiguration("perception_required_samples")
    perception_minimum_span_m = LaunchConfiguration("perception_minimum_span_m")
    perception_skip_initial_samples = LaunchConfiguration(
        "perception_skip_initial_samples"
    )
    enable_motion_prediction = LaunchConfiguration("enable_motion_prediction")
    prediction_horizon_s = LaunchConfiguration("prediction_horizon_s")
    enable_grasp_candidate_prevalidation = LaunchConfiguration(
        "enable_grasp_candidate_prevalidation"
    )
    defer_grasp_candidate_prevalidation_until_dynamic_replan = LaunchConfiguration(
        "defer_grasp_candidate_prevalidation_until_dynamic_replan"
    )
    package_share = get_package_share_directory("panda_manipulation")
    rgbd_rviz_config = os.path.join(
        package_share,
        "config",
        "rgbd_pointcloud.rviz",
    )
    pick_launch = os.path.join(package_share, "launch", "gazebo_pick.launch.py")
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "planner_id",
                default_value="RRTConnectkConfigDefault",
            ),
            DeclareLaunchArgument("obstacle_start_y", default_value="-0.55"),
            DeclareLaunchArgument("obstacle_end_y", default_value="0.55"),
            DeclareLaunchArgument("obstacle_speed_mps", default_value="0.14"),
            DeclareLaunchArgument("obstacle_trigger_delay_s", default_value="0.2"),
            DeclareLaunchArgument("perception_required_samples", default_value="12"),
            DeclareLaunchArgument("perception_minimum_span_m", default_value="0.12"),
            DeclareLaunchArgument("perception_skip_initial_samples", default_value="5"),
            DeclareLaunchArgument("enable_motion_prediction", default_value="true"),
            DeclareLaunchArgument("prediction_horizon_s", default_value="0.65"),
            DeclareLaunchArgument(
                "enable_grasp_candidate_prevalidation",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "defer_grasp_candidate_prevalidation_until_dynamic_replan",
                default_value="true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pick_launch),
                launch_arguments={
                    "gui": gui,
                    "rviz": rviz,
                    "rviz_config": rgbd_rviz_config,
                    "target_x": "0.52",
                    "target_y": "0.0",
                    "place_x": "0.50",
                    "place_y": "0.12",
                    "planner_ids_csv": planner_id,
                    "pre_grasp_orientation_tolerance": "0.12",
                    "use_camera_perception": "true",
                    "use_ompl_for_place": "true",
                    "ompl_execution_backend": "execute_trajectory",
                    "enable_dynamic_replanning": "true",
                    "maximum_dynamic_replans": "3",
                    "enable_grasp_candidate_prevalidation": (
                        enable_grasp_candidate_prevalidation
                    ),
                    "defer_grasp_candidate_prevalidation_until_dynamic_replan": (
                        defer_grasp_candidate_prevalidation_until_dynamic_replan
                    ),
                }.items(),
            ),
            Node(
                package="panda_manipulation",
                executable="depth_obstacle_tracker",
                name="depth_obstacle_tracker",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "collision_padding_m": 0.06,
                        "missing_detection_limit": 8,
                        "enable_motion_prediction": enable_motion_prediction,
                        "prediction_horizon_s": prediction_horizon_s,
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="trajectory_safety_monitor",
                name="trajectory_safety_monitor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "scene_settle_s": 0.20,
                        "maximum_state_samples": 30,
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="dynamic_obstacle_controller",
                name="dynamic_obstacle_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "trigger_step": "ompl_to_pre_grasp",
                        "trigger_mode": "executing",
                        "trigger_delay_s": obstacle_trigger_delay_s,
                        "start_y": obstacle_start_y,
                        "end_y": obstacle_end_y,
                        "speed_mps": obstacle_speed_mps,
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="dynamic_obstacle_perception_validator",
                name="dynamic_obstacle_perception_validator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "required_samples": perception_required_samples,
                        "minimum_tracking_span_m": perception_minimum_span_m,
                        "skip_initial_tracking_samples": perception_skip_initial_samples,
                    }
                ],
            ),
        ]
    )
