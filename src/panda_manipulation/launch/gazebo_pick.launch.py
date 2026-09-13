import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    gui_config = LaunchConfiguration("gui_config")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    world = LaunchConfiguration("world")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    place_x = LaunchConfiguration("place_x")
    place_y = LaunchConfiguration("place_y")
    planner_ids_csv = LaunchConfiguration("planner_ids_csv")
    ompl_velocity_scaling_factor = LaunchConfiguration(
        "ompl_velocity_scaling_factor"
    )
    payload_transfer_velocity_scaling_factor = LaunchConfiguration(
        "payload_transfer_velocity_scaling_factor"
    )
    place_descent_velocity_scaling_factor = LaunchConfiguration(
        "place_descent_velocity_scaling_factor"
    )
    place_descent_recovery_velocity_scaling_factor = LaunchConfiguration(
        "place_descent_recovery_velocity_scaling_factor"
    )
    place_descent_stage_count = LaunchConfiguration("place_descent_stage_count")
    place_descent_backend = LaunchConfiguration("place_descent_backend")
    servo_fault_injection_type = LaunchConfiguration(
        "servo_fault_injection_type"
    )
    servo_fault_injection_stage = LaunchConfiguration(
        "servo_fault_injection_stage"
    )
    servo_fault_injection_delay_s = LaunchConfiguration(
        "servo_fault_injection_delay_s"
    )
    enable_physical_disturbance = LaunchConfiguration(
        "enable_physical_disturbance"
    )
    physical_disturbance_stage = LaunchConfiguration(
        "physical_disturbance_stage"
    )
    physical_disturbance_delay_s = LaunchConfiguration(
        "physical_disturbance_delay_s"
    )
    physical_disturbance_duration_s = LaunchConfiguration(
        "physical_disturbance_duration_s"
    )
    physical_disturbance_force_x_n = LaunchConfiguration(
        "physical_disturbance_force_x_n"
    )
    physical_disturbance_force_y_n = LaunchConfiguration(
        "physical_disturbance_force_y_n"
    )
    physical_disturbance_force_z_n = LaunchConfiguration(
        "physical_disturbance_force_z_n"
    )
    pre_place_joint_replay_positions_csv = LaunchConfiguration(
        "pre_place_joint_replay_positions_csv"
    )
    obstacle_x = LaunchConfiguration("obstacle_x")
    obstacle_y = LaunchConfiguration("obstacle_y")
    obstacle_z = LaunchConfiguration("obstacle_z")
    obstacle_size_x = LaunchConfiguration("obstacle_size_x")
    obstacle_size_y = LaunchConfiguration("obstacle_size_y")
    obstacle_size_z = LaunchConfiguration("obstacle_size_z")
    diagnose_direct_path = LaunchConfiguration("diagnose_direct_path")
    require_direct_path_blocked = LaunchConfiguration("require_direct_path_blocked")
    use_ompl_for_place = LaunchConfiguration("use_ompl_for_place")
    use_cartesian_pre_place_transfer = LaunchConfiguration(
        "use_cartesian_pre_place_transfer"
    )
    pre_grasp_orientation_tolerance = LaunchConfiguration(
        "pre_grasp_orientation_tolerance"
    )
    use_camera_perception = LaunchConfiguration("use_camera_perception")
    use_rgbd_target_perception = LaunchConfiguration(
        "use_rgbd_target_perception"
    )
    marker_size_m = LaunchConfiguration("marker_size_m")
    object_center_offset_z_m = LaunchConfiguration(
        "object_center_offset_z_m"
    )
    grasp_preclose_max_target_drift_m = LaunchConfiguration(
        "grasp_preclose_max_target_drift_m"
    )
    grasp_height_offset = LaunchConfiguration("grasp_height_offset")
    gripper_closed_position = LaunchConfiguration("gripper_closed_position")
    enable_tactile_grasp_supervision = LaunchConfiguration(
        "enable_tactile_grasp_supervision"
    )
    enable_physical_grasp_verification = LaunchConfiguration(
        "enable_physical_grasp_verification"
    )
    release_clearance_height = LaunchConfiguration("release_clearance_height")
    ompl_execution_backend = LaunchConfiguration("ompl_execution_backend")
    enable_dynamic_replanning = LaunchConfiguration("enable_dynamic_replanning")
    maximum_dynamic_replans = LaunchConfiguration("maximum_dynamic_replans")
    enable_grasp_candidate_prevalidation = LaunchConfiguration(
        "enable_grasp_candidate_prevalidation"
    )
    grasp_candidate_prevalidation_max_rounds = LaunchConfiguration(
        "grasp_candidate_prevalidation_max_rounds"
    )
    restrict_grasp_yaw_to_nominal = LaunchConfiguration(
        "restrict_grasp_yaw_to_nominal"
    )
    force_grasp_yaw_offset = LaunchConfiguration("force_grasp_yaw_offset")
    forced_grasp_yaw_offset_deg = LaunchConfiguration(
        "forced_grasp_yaw_offset_deg"
    )
    defer_grasp_candidate_prevalidation_until_dynamic_replan = LaunchConfiguration(
        "defer_grasp_candidate_prevalidation_until_dynamic_replan"
    )
    enable_transfer_diagnostics = LaunchConfiguration(
        "enable_transfer_diagnostics"
    )
    transfer_diagnostics_csv_path = LaunchConfiguration(
        "transfer_diagnostics_csv_path"
    )
    transfer_diagnostics_summary_path = LaunchConfiguration(
        "transfer_diagnostics_summary_path"
    )
    transfer_diagnostics_label = LaunchConfiguration(
        "transfer_diagnostics_label"
    )
    package_share = get_package_share_directory("panda_manipulation")
    panda_moveit_share = get_package_share_directory(
        "moveit_resources_panda_moveit_config"
    )
    default_world = os.path.join(package_share, "worlds", "panda_table.sdf")
    default_gui_config = os.path.join(
        package_share,
        "config",
        "gazebo_demo.gui.config",
    )
    moveit_gazebo_launch = os.path.join(
        package_share,
        "launch",
        "moveit_gazebo.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("gui_config", default_value=default_gui_config),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(panda_moveit_share, "launch", "moveit.rviz"),
            ),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("target_x", default_value="0.52"),
            DeclareLaunchArgument("target_y", default_value="0.0"),
            DeclareLaunchArgument("place_x", default_value="0.45"),
            DeclareLaunchArgument("place_y", default_value="-0.16"),
            DeclareLaunchArgument("obstacle_x", default_value="0.42"),
            DeclareLaunchArgument("obstacle_y", default_value="0.30"),
            DeclareLaunchArgument("obstacle_z", default_value="0.09"),
            DeclareLaunchArgument("obstacle_size_x", default_value="0.12"),
            DeclareLaunchArgument("obstacle_size_y", default_value="0.12"),
            DeclareLaunchArgument("obstacle_size_z", default_value="0.18"),
            DeclareLaunchArgument("static_environment_world", default_value=""),
            DeclareLaunchArgument("pre_place_candidate_height_step_m", default_value="0.0"),
            DeclareLaunchArgument("cube_symmetric_placement", default_value="false"),
            DeclareLaunchArgument("diagnose_rejected_descent", default_value="false"),
            DeclareLaunchArgument("enable_place_endpoint_precheck", default_value="false"),
            DeclareLaunchArgument("enable_loaded_path_precheck", default_value="false"),
            DeclareLaunchArgument("grasp_pitch_offset_deg", default_value="0.0"),
            DeclareLaunchArgument("diagnose_direct_path", default_value="false"),
            DeclareLaunchArgument("require_direct_path_blocked", default_value="false"),
            DeclareLaunchArgument("use_ompl_for_place", default_value="false"),
            DeclareLaunchArgument(
                "use_cartesian_pre_place_transfer",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "use_camera_perception",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "use_rgbd_target_perception",
                default_value="false",
            ),
            # Calibrated effective edge for the rendered 45 mm marker.
            DeclareLaunchArgument("marker_size_m", default_value="0.044"),
            DeclareLaunchArgument(
                "object_center_offset_z_m",
                default_value="-0.04",
            ),
            DeclareLaunchArgument(
                "grasp_preclose_max_target_drift_m",
                default_value="0.012",
            ),
            DeclareLaunchArgument(
                "grasp_height_offset",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "gripper_closed_position",
                default_value="0.022",
            ),
            DeclareLaunchArgument(
                "enable_tactile_grasp_supervision",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_physical_grasp_verification",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "release_clearance_height",
                default_value="0.12",
            ),
            DeclareLaunchArgument(
                "pre_grasp_orientation_tolerance",
                default_value="0.35",
            ),
            DeclareLaunchArgument("ompl_execution_backend", default_value="move_group"),
            DeclareLaunchArgument("enable_dynamic_replanning", default_value="false"),
            DeclareLaunchArgument("maximum_dynamic_replans", default_value="2"),
            DeclareLaunchArgument(
                "ompl_velocity_scaling_factor",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "payload_transfer_velocity_scaling_factor",
                default_value="0.03",
            ),
            DeclareLaunchArgument(
                "place_descent_velocity_scaling_factor",
                default_value="0.01",
            ),
            DeclareLaunchArgument(
                "place_descent_recovery_velocity_scaling_factor",
                default_value="0.02",
            ),
            DeclareLaunchArgument(
                "place_descent_stage_count",
                default_value="6",
            ),
            DeclareLaunchArgument(
                "place_descent_backend",
                default_value="cartesian",
                choices=["cartesian", "servo"],
            ),
            DeclareLaunchArgument(
                "servo_fault_injection_type",
                default_value="none",
                choices=["none", "contact_loss", "payload_drift"],
            ),
            DeclareLaunchArgument(
                "servo_fault_injection_stage",
                default_value="2",
            ),
            DeclareLaunchArgument(
                "servo_fault_injection_delay_s",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "enable_physical_disturbance",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "physical_disturbance_stage",
                default_value="3",
            ),
            DeclareLaunchArgument(
                "physical_disturbance_delay_s",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "physical_disturbance_duration_s",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "physical_disturbance_force_x_n",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "physical_disturbance_force_y_n",
                default_value="2.0",
            ),
            DeclareLaunchArgument(
                "physical_disturbance_force_z_n",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "pre_place_joint_replay_positions_csv",
                default_value="",
            ),
            DeclareLaunchArgument(
                "enable_grasp_candidate_prevalidation",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "grasp_candidate_prevalidation_max_rounds",
                default_value="2",
            ),
            DeclareLaunchArgument(
                "restrict_grasp_yaw_to_nominal",
                default_value="false",
            ),
            DeclareLaunchArgument("force_grasp_yaw_offset", default_value="false"),
            DeclareLaunchArgument(
                "forced_grasp_yaw_offset_deg",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "defer_grasp_candidate_prevalidation_until_dynamic_replan",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "enable_transfer_diagnostics",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "transfer_diagnostics_csv_path",
                default_value="/tmp/panda_transfer_diagnostics.csv",
            ),
            DeclareLaunchArgument(
                "transfer_diagnostics_summary_path",
                default_value="/tmp/panda_transfer_diagnostics.json",
            ),
            DeclareLaunchArgument(
                "transfer_diagnostics_label",
                default_value="loaded",
            ),
            DeclareLaunchArgument(
                "planner_ids_csv",
                default_value=(
                    "RRTConnectkConfigDefault,"
                    "PRMkConfigDefault,"
                    "RRTstarkConfigDefault"
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_gazebo_launch),
                launch_arguments={
                    "gui": gui,
                    "gui_config": gui_config,
                    "rviz": rviz,
                    "rviz_config": rviz_config,
                    "world": world,
                    "run_smoke_test": "false",
                    "static_environment_world": LaunchConfiguration("static_environment_world"),
                    "enable_servo": PythonExpression(
                        ["'", place_descent_backend, "' == 'servo'"]
                    ),
                    "obstacle_x": obstacle_x,
                    "obstacle_y": obstacle_y,
                    "obstacle_z": obstacle_z,
                    "obstacle_size_x": obstacle_size_x,
                    "obstacle_size_y": obstacle_size_y,
                    "obstacle_size_z": obstacle_size_z,
                }.items(),
            ),
            Node(
                package="panda_manipulation",
                executable="aruco_pose_estimator",
                name="gazebo_target_pose",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_camera_perception,
                            "'.lower() != 'true' and '",
                            use_rgbd_target_perception,
                            "'.lower() != 'true'",
                        ]
                    )
                ),
                parameters=[
                    {
                        "use_synthetic_pose": True,
                        "base_frame": "panda_link0",
                        "target_x_m": target_x,
                        "target_y_m": target_y,
                        "target_z_m": 0.04,
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="aruco_perception_validator",
                name="gazebo_aruco_perception_validator",
                output="screen",
                condition=IfCondition(use_camera_perception),
                parameters=[
                    {
                        "use_sim_time": True,
                        "required_samples": 10,
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="aruco_pose_estimator",
                name="gazebo_aruco_pose_estimator",
                output="screen",
                condition=IfCondition(use_camera_perception),
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
                executable="rgbd_target_pose_estimator",
                name="gazebo_rgbd_target_pose_estimator",
                output="screen",
                condition=IfCondition(use_rgbd_target_perception),
                parameters=[
                    {
                        "use_sim_time": True,
                        "base_frame": "panda_link0",
                        "object_height_m": 0.08,
                        "minimum_red_area_px": 80.0,
                        "minimum_depth_points": 3,
                        # Fixed overhead-camera calibration from Gazebo truth.
                        # Keep estimator defaults at zero for real-camera use.
                        "rgb_x_bias_m": -0.00465,
                        "rgb_y_bias_m": -0.00050,
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="aruco_perception_validator",
                name="gazebo_rgbd_perception_validator",
                output="screen",
                condition=IfCondition(use_rgbd_target_perception),
                parameters=[
                    {
                        "use_sim_time": True,
                        "required_samples": 10,
                        "status_topic": "/rgbd_target_detection",
                        "result_topic": "/rgbd_target_perception_validation",
                        "source_name": "RGB-D markerless",
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="grasp_pose_generator",
                name="gazebo_grasp_pose_generator",
                output="screen",
                parameters=[
                    {
                        "place_x": place_x,
                        "place_y": place_y,
                        # Keep an 8 mm table clearance before the staged
                        # release. This avoids both closed-gripper table drag
                        # and the unstable 15 mm free drop used previously.
                        "place_z": 0.153,
                        # Top-down grasp: stay centered in X/Y and descend from
                        # above so the open fingers surround rather than push the cube.
                        "approach_distance": 0.0,
                        # panda_hand is the palm frame. At 0.10 m above the cube
                        # center, the finger collision meshes overlap its side faces
                        # instead of touching only the upper edge.
                        "grasp_height_offset": grasp_height_offset,
                        "lift_distance": 0.10,
                        "pre_grasp_height_offset": 0.08,
                        # Lift vertically after release. A horizontal retreat can
                        # sweep the open fingers through the newly placed cube.
                        "retreat_height_offset": 0.08,
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="manipulation_marker_publisher",
                name="gazebo_manipulation_markers",
                output="screen",
            ),
            Node(
                package="panda_manipulation",
                executable="target_object_manager",
                name="gazebo_target_object_manager",
                output="screen",
                parameters=[
                    {
                        "manage_collision_object": True,
                        "publish_period_s": 0.2,
                        "use_live_release_pose": ParameterValue(
                            PythonExpression(
                                [
                                    "'",
                                    use_camera_perception,
                                    "'.lower() == 'true' or '",
                                    use_rgbd_target_perception,
                                    "'.lower() == 'true'",
                                ]
                            ),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="pick_plan_pipeline",
                name="gazebo_pick_pipeline",
                output="screen",
                parameters=[
                    {
                        "plan_once": True,
                        "execute_trajectories": True,
                        "visualize_only_execution": False,
                        "ompl_execution_backend": ompl_execution_backend,
                        "enable_dynamic_replanning": enable_dynamic_replanning,
                        "maximum_dynamic_replans": maximum_dynamic_replans,
                        "enable_grasp_candidate_prevalidation": (
                            enable_grasp_candidate_prevalidation
                        ),
                        "restrict_grasp_yaw_to_nominal": ParameterValue(
                            restrict_grasp_yaw_to_nominal,
                            value_type=bool,
                        ),
                        "force_grasp_yaw_offset": ParameterValue(
                            force_grasp_yaw_offset,
                            value_type=bool,
                        ),
                        "forced_grasp_yaw_offset_deg": ParameterValue(
                            forced_grasp_yaw_offset_deg,
                            value_type=float,
                        ),
                        "grasp_candidate_prevalidation_min_fraction": 0.98,
                        "grasp_candidate_prevalidation_max_rounds": ParameterValue(
                            grasp_candidate_prevalidation_max_rounds,
                            value_type=int,
                        ),
                        "grasp_candidate_prevalidation_scene_settle_s": 0.5,
                        "defer_grasp_candidate_prevalidation_until_dynamic_replan": (
                            defer_grasp_candidate_prevalidation_until_dynamic_replan
                        ),
                        "execute_gripper": True,
                        "gripper_execution_backend": "follow_joint_trajectory",
                        "gripper_action_name": (
                            "/panda_hand_controller/follow_joint_trajectory"
                        ),
                        "gripper_open_position": 0.04,
                        # The 50 mm cube contacts near 0.025 m per finger. A 0.022 m
                        # target provides mild preload without a high-force overclose.
                        "gripper_closed_position": gripper_closed_position,
                        "enable_tactile_grasp_supervision": ParameterValue(
                            enable_tactile_grasp_supervision,
                            value_type=bool,
                        ),
                        "target_contact_topic": "/gazebo/target_cube/contacts",
                        "tactile_left_finger_token": "panda_leftfinger",
                        "tactile_right_finger_token": "panda_rightfinger",
                        "tactile_contact_settle_s": 0.2,
                        "tactile_contact_max_age_s": 0.75,
                        "tactile_release_max_retries": 1,
                        "release_object_max_horizontal_motion_m": 0.05,
                        "release_object_max_vertical_motion_m": 0.06,
                        "gripper_release_position": 0.028,
                        "gripper_release_settle_s": 0.5,
                        "gripper_motion_duration_s": 2.0,
                        "gripper_free_motion_max_retries": 1,
                        # A close command that times out without moving the
                        # fingers is retried in place before moving the arm.
                        "gripper_close_in_place_max_retries": 1,
                        "gripper_close_min_movement_m": 0.002,
                        "enable_physical_grasp_verification": ParameterValue(
                            enable_physical_grasp_verification,
                            value_type=bool,
                        ),
                        "physical_target_pose_topic": "/gazebo/target_pose",
                        "grasp_probe_lift_m": 0.03,
                        "grasp_probe_min_object_lift_m": 0.015,
                        "grasp_probe_settle_s": 0.5,
                        "release_settle_s": 2.0,
                        # Leave the released cube using a straight, orientation-
                        # preserving lift before any global return motion.
                        "release_clearance_height": release_clearance_height,
                        "pre_place_height_offset": 0.12,
                        "pre_place_candidate_attempts": 4,
                        "diagnose_rejected_descent": ParameterValue(
                            LaunchConfiguration("diagnose_rejected_descent"), value_type=bool
                        ),
                        "enable_place_endpoint_precheck": ParameterValue(
                            LaunchConfiguration("enable_place_endpoint_precheck"), value_type=bool
                        ),
                        "enable_loaded_path_precheck": ParameterValue(
                            LaunchConfiguration("enable_loaded_path_precheck"), value_type=bool
                        ),
                        "grasp_pitch_offset_deg": ParameterValue(
                            LaunchConfiguration("grasp_pitch_offset_deg"), value_type=float
                        ),
                        "cube_symmetric_placement": ParameterValue(
                            LaunchConfiguration("cube_symmetric_placement"), value_type=bool
                        ),
                        "pre_place_candidate_height_step_m": ParameterValue(
                            LaunchConfiguration("pre_place_candidate_height_step_m"), value_type=float
                        ),
                        "pre_place_joint_replay_positions_csv": (
                            pre_place_joint_replay_positions_csv
                        ),
                        "pre_place_joint_replay_tolerance_rad": 0.005,
                        "direct_place_candidate_attempts": 4,
                        "place_transfer_max_joint_path_length": 4.5,
                        "payload_transfer_tolerance_m": 0.06,
                        "place_object_clearance_m": 0.008,
                        "place_feedback_tolerance_m": 0.05,
                        "place_feedback_max_correction_m": 0.12,
                        "place_feedback_max_retries": 1,
                        "place_feedback_settle_s": 0.5,
                        # Never drag a misplaced object across the table. Lift
                        # it clear, translate while suspended, then descend.
                        "place_feedback_recovery_height_m": 0.05,
                        "place_feedback_min_improvement_m": 0.005,
                        # Short measured-state segments limit lateral drift and
                        # stop at the first sign of payload slip before release.
                        "place_descent_stage_count": ParameterValue(
                            place_descent_stage_count,
                            value_type=int,
                        ),
                        "place_descent_backend": place_descent_backend,
                        "servo_descent_speed_mps": 0.01,
                        "servo_command_period_s": 0.02,
                        "servo_stage_position_tolerance_m": 0.002,
                        "servo_stage_timeout_margin_s": 2.0,
                        "servo_collision_limited_acceptance_m": 0.012,
                        "enable_servo_cartesian_fallback": True,
                        "servo_fault_injection_type": (
                            servo_fault_injection_type
                        ),
                        "servo_fault_injection_stage": ParameterValue(
                            servo_fault_injection_stage,
                            value_type=int,
                        ),
                        "servo_fault_injection_delay_s": ParameterValue(
                            servo_fault_injection_delay_s,
                            value_type=float,
                        ),
                        "place_descent_stage_max_horizontal_motion_m": 0.01,
                        "place_descent_stage_payload_tolerance_m": 0.015,
                        "place_descent_endpoint_orientation_tolerance_rad": 0.10,
                        # Wait for Gazebo joint/TF state to converge to the
                        # OMPL endpoint before beginning contact descent.
                        "place_descent_start_tolerance_m": 0.01,
                        "place_descent_start_poll_s": 0.2,
                        "place_descent_start_timeout_s": 3.0,
                        "place_descent_start_max_alignment_retries": 2,
                        "payload_transfer_velocity_scaling_factor": ParameterValue(
                            payload_transfer_velocity_scaling_factor,
                            value_type=float,
                        ),
                        "place_descent_velocity_scaling_factor": ParameterValue(
                            place_descent_velocity_scaling_factor,
                            value_type=float,
                        ),
                        "place_descent_recovery_velocity_scaling_factor": ParameterValue(
                            place_descent_recovery_velocity_scaling_factor,
                            value_type=float,
                        ),
                        "allow_ompl_release_retreat_fallback": True,
                        "release_retreat_max_joint_path_length": 1.5,
                        "return_home_after_success": True,
                        "return_home_scene_settle_s": 1.0,
                        # A rejected plan-only return path leaves the robot
                        # unchanged, so retry stochastic OMPL sampling without
                        # weakening collision validation.
                        "return_home_max_retries": 3,
                        # Allow the target collision-object removal to reach
                        # move_group before contact approach planning starts.
                        "approach_scene_settle_s": 0.5,
                        "grasp_probe_max_retries": 2,
                        "preclose_recenter_max_retries": 2,
                        "grasp_preclose_max_target_drift_m": (
                            grasp_preclose_max_target_drift_m
                        ),
                        "grasp_recovery_max_translation_m": 0.10,
                        # Unequal dual-finger contact estimates lateral object
                        # offset and recenters the next grasp in the hand frame.
                        "grasp_finger_asymmetry_correction_gain": 1.0,
                        "grasp_finger_asymmetry_min_m": 0.001,
                        "grasp_finger_max_correction_m": 0.004,
                        "use_fixed_home_start": False,
                        "reset_to_home_before_execute": False,
                        "fail_on_start_state_mismatch": True,
                        "execution_start_tolerance": 0.05,
                        "joint_state_wait_timeout_s": 10.0,
                        # OMPL can occasionally sample a path that MoveIt's final
                        # dense collision validation rejects. Replan from the same
                        # unchanged state; never execute the rejected trajectory.
                        "ompl_invalid_plan_retries": 2,
                        # OMPL must finish close enough that the final alignment
                        # remains a short, safe motion above the target.
                        "position_tolerance": 0.01,
                        "cartesian_min_fraction": 0.7,
                        "cartesian_avoid_collisions": True,
                        "diagnose_direct_path_to_pre_grasp": diagnose_direct_path,
                        "require_direct_path_blocked": require_direct_path_blocked,
                        "use_ompl_for_place": use_ompl_for_place,
                        "use_cartesian_pre_place_transfer": (
                            use_cartesian_pre_place_transfer
                        ),
                        "cartesian_velocity_scaling_factor": 0.12,
                        "cartesian_acceleration_scaling_factor": 0.12,
                        # Final contact approach is slower than lift and transfer
                        # motion so the open fingers do not strike a free object.
                        "contact_approach_velocity_scaling_factor": 0.05,
                        "contact_approach_acceleration_scaling_factor": 0.05,
                        "enable_cartesian_approach_recovery": True,
                        "approach_recovery_ratio": 0.5,
                        "pre_grasp_orientation_tolerance": (
                            pre_grasp_orientation_tolerance
                        ),
                        "planner_ids_csv": planner_ids_csv,
                        "ompl_velocity_scaling_factor": ParameterValue(
                            ompl_velocity_scaling_factor,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="physical_disturbance_injector",
                name="physical_disturbance_injector",
                output="screen",
                condition=IfCondition(enable_physical_disturbance),
                parameters=[
                    {
                        "use_sim_time": True,
                        "trigger_stage": ParameterValue(
                            physical_disturbance_stage,
                            value_type=int,
                        ),
                        "trigger_delay_s": ParameterValue(
                            physical_disturbance_delay_s,
                            value_type=float,
                        ),
                        "duration_s": ParameterValue(
                            physical_disturbance_duration_s,
                            value_type=float,
                        ),
                        "force_x_n": ParameterValue(
                            physical_disturbance_force_x_n,
                            value_type=float,
                        ),
                        "force_y_n": ParameterValue(
                            physical_disturbance_force_y_n,
                            value_type=float,
                        ),
                        "force_z_n": ParameterValue(
                            physical_disturbance_force_z_n,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="panda_manipulation",
                executable="gazebo_pick_validator",
                name="gazebo_pick_validator",
                output="screen",
                parameters=[{"place_x": place_x, "place_y": place_y}],
            ),
            Node(
                package="panda_manipulation",
                executable="transfer_diagnostics_recorder",
                name="transfer_diagnostics_recorder",
                output="screen",
                condition=IfCondition(enable_transfer_diagnostics),
                parameters=[
                    {
                        "use_sim_time": True,
                        "csv_path": transfer_diagnostics_csv_path,
                        "summary_path": transfer_diagnostics_summary_path,
                        "label": transfer_diagnostics_label,
                        "payload_drift_threshold_m": 0.015,
                    }
                ],
            ),
        ]
    )
