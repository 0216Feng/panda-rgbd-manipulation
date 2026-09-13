import os
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class GazeboAssetTests(unittest.TestCase):
    def test_gazebo_demo_view_is_installed_and_forwarded(self):
        config_path = os.path.join(
            PACKAGE_ROOT,
            "config",
            "gazebo_demo.gui.config",
        )
        with open(config_path, encoding="utf-8") as config_file:
            config_text = config_file.read()
        self.assertIn("<width>1280</width>", config_text)
        self.assertIn("<height>720</height>", config_text)
        self.assertIn(
            "<camera_pose>-2.4 -2.0 2.0 0 0.35 0.65</camera_pose>",
            config_text,
        )

        for launch_name in (
            "gazebo_control.launch.py",
            "moveit_gazebo.launch.py",
            "gazebo_pick.launch.py",
        ):
            launch_path = os.path.join(PACKAGE_ROOT, "launch", launch_name)
            with open(launch_path, encoding="utf-8") as launch_file:
                launch_text = launch_file.read()
            self.assertIn('LaunchConfiguration("gui_config")', launch_text)
            self.assertIn('DeclareLaunchArgument("gui_config"', launch_text)

    def test_world_is_valid_xml_and_contains_required_models(self):
        world_path = os.path.join(PACKAGE_ROOT, "worlds", "panda_table.sdf")
        root = ET.parse(world_path).getroot()
        model_names = {model.attrib["name"] for model in root.findall(".//model")}
        self.assertTrue(
            {
                "robot_pedestal",
                "work_table",
                "target_cube",
                "obstacle_block",
                "dynamic_obstacle",
            }.issubset(model_names)
        )

        sensors = {
            sensor.attrib.get("name"): sensor.attrib.get("type")
            for sensor in root.findall(".//sensor")
        }
        self.assertEqual(sensors.get("depth_camera"), "rgbd_camera")
        self.assertEqual(sensors.get("showcase_rgb_camera"), "camera")
        showcase_camera = root.find(
            ".//model[@name='showcase_camera']/link/"
            "sensor[@name='showcase_rgb_camera']"
        )
        showcase_model = root.find(".//model[@name='showcase_camera']")
        self.assertEqual(
            showcase_model.findtext("pose"),
            "1.80 1.25 1.25 0 0.22 -2.40",
        )
        self.assertEqual(showcase_camera.findtext("always_on"), "false")
        self.assertEqual(
            showcase_camera.findtext("topic"),
            "/showcase_camera/image_raw",
        )
        self.assertEqual(
            showcase_camera.findtext("camera/image/width"),
            "640",
        )
        self.assertEqual(
            showcase_camera.findtext("camera/image/height"),
            "360",
        )
        self.assertEqual(sensors.get("target_cube_contact"), "contact")
        target_contact = root.find(
            ".//model[@name='target_cube']/link/sensor[@name='target_cube_contact']"
        )
        self.assertEqual(target_contact.findtext("contact/collision"), "collision")
        system_plugins = {
            plugin.attrib.get("name") for plugin in root.findall("./world/plugin")
        }
        self.assertIn("gz::sim::systems::Contact", system_plugins)
        self.assertIn("gz::sim::systems::ApplyLinkWrench", system_plugins)

    def test_bridge_exposes_depth_cloud_and_dynamic_obstacle_pose(self):
        bridge_path = os.path.join(PACKAGE_ROOT, "config", "gazebo_bridge.yaml")
        with open(bridge_path, encoding="utf-8") as bridge_file:
            bridge_text = bridge_file.read()
        self.assertIn("/camera/depth/points", bridge_text)
        self.assertIn("gz.msgs.PointCloudPacked", bridge_text)
        self.assertIn("/gazebo/dynamic_obstacle_pose", bridge_text)
        self.assertIn("/gazebo/target_cube/contacts", bridge_text)
        self.assertIn("/gazebo/target_wrench/persistent", bridge_text)
        self.assertIn("/gazebo/target_wrench/clear", bridge_text)
        self.assertIn("ros_gz_interfaces/msg/EntityWrench", bridge_text)
        self.assertEqual(bridge_text.count("direction: ROS_TO_GZ"), 2)
        self.assertIn(
            "/world/panda_table/model/target_cube/link/target_link/"
            "sensor/target_cube_contact/contact",
            bridge_text,
        )
        self.assertEqual(bridge_text.count("ros_gz_interfaces/msg/Contacts"), 1)

        launch_path = os.path.join(PACKAGE_ROOT, "launch", "gazebo_control.launch.py")
        with open(launch_path, encoding="utf-8") as launch_file:
            launch_text = launch_file.read()
        self.assertIn("SetEntityPose", launch_text)
        self.assertIn("overhead_depth_camera_frame", launch_text)

        pick_launch_path = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "gazebo_pick.launch.py",
        )
        with open(pick_launch_path, encoding="utf-8") as launch_file:
            pick_launch_text = launch_file.read()
        self.assertIn('executable="physical_disturbance_injector"', pick_launch_text)
        self.assertIn('"enable_physical_disturbance"', pick_launch_text)
        self.assertIn('"physical_disturbance_stage"', pick_launch_text)

        perception_launch = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "dynamic_obstacle_perception.launch.py",
        )
        with open(perception_launch, encoding="utf-8") as launch_file:
            perception_text = launch_file.read()
        self.assertIn('executable="depth_obstacle_tracker"', perception_text)

    def test_rgbd_rviz_config_displays_point_cloud(self):
        config_path = os.path.join(
            PACKAGE_ROOT,
            "config",
            "rgbd_pointcloud.rviz",
        )
        with open(config_path, encoding="utf-8") as config_file:
            config_text = config_file.read()
        self.assertIn("Class: rviz_default_plugins/PointCloud2", config_text)
        self.assertIn("Value: /camera/depth/points", config_text)
        self.assertIn("Fixed Frame: panda_link0", config_text)

        focused_config_path = os.path.join(
            PACKAGE_ROOT,
            "config",
            "rgbd_pointcloud_only.rviz",
        )
        with open(focused_config_path, encoding="utf-8") as config_file:
            focused_config = config_file.read()
        self.assertIn("Class: rviz_default_plugins/PointCloud2", focused_config)
        self.assertIn("Value: /camera/depth/points", focused_config)
        self.assertIn("Value: /rgbd_target/points", focused_config)
        self.assertIn("Name: Markerless Target Pose", focused_config)

        focused_launch_path = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "rgbd_pointcloud_visualization.launch.py",
        )
        with open(focused_launch_path, encoding="utf-8") as launch_file:
            focused_launch = launch_file.read()
        self.assertIn('"--fullscreen"', focused_launch)
        self.assertIn('"rviz": "false"', focused_launch)

        markerless_launch_path = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "rgbd_markerless_pick.launch.py",
        )
        with open(markerless_launch_path, encoding="utf-8") as launch_file:
            markerless_launch = launch_file.read()
        self.assertIn('"use_rgbd_target_perception": "true"', markerless_launch)
        self.assertIn("rgbd_pointcloud_only.rviz", markerless_launch)
        self.assertIn('"grasp_height_offset": "0.105"', markerless_launch)
        self.assertIn('"use_ompl_for_place": "true"', markerless_launch)
        self.assertIn(
            '"use_cartesian_pre_place_transfer": "true"',
            markerless_launch,
        )
        self.assertIn('"gripper_closed_position": "0.022"', markerless_launch)
        self.assertIn('"place_y": "-0.080"', markerless_launch)

    def test_robot_xacro_declares_gazebo_ros2_control(self):
        xacro_path = os.path.join(PACKAGE_ROOT, "urdf", "panda_gz.urdf.xacro")
        root = ET.parse(xacro_path).getroot()
        control = root.find("ros2_control")
        self.assertIsNotNone(control)
        plugin = control.find("./hardware/plugin")
        self.assertEqual(plugin.text, "gz_ros2_control/GazeboSimSystem")
        controlled_joints = {joint.attrib["name"] for joint in control.findall("joint")}
        self.assertTrue({f"panda_joint{index}" for index in range(1, 8)}.issubset(controlled_joints))
        finger2 = control.find("./joint[@name='panda_finger_joint2']")
        effort_command = finger2.find("./command_interface[@name='effort']")
        self.assertIsNotNone(effort_command)
        self.assertEqual(effort_command.findtext("./param[@name='min']"), "-20.0")
        self.assertEqual(effort_command.findtext("./param[@name='max']"), "20.0")
        self.assertIsNone(finger2.find("./command_interface[@name='position']"))
        self.assertIsNotNone(finger2.find("./state_interface[@name='position']"))
        self.assertIsNotNone(finger2.find("./state_interface[@name='velocity']"))
        self.assertIsNotNone(finger2.find("./state_interface[@name='effort']"))
        gazebo_plugin = root.find(
            "./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']"
        )
        self.assertIsNotNone(gazebo_plugin)
        self.assertEqual(
            gazebo_plugin.findtext("position_proportional_gain"),
            "1.0",
        )

    def test_vendored_panda_model_has_symmetric_finger_inertia(self):
        model_path = os.path.join(
            PACKAGE_ROOT,
            "urdf",
            "panda_balanced.urdf.xacro",
        )
        root = ET.parse(model_path).getroot()
        masses = []
        for link_name in ("panda_leftfinger", "panda_rightfinger"):
            mass = root.find(
                f"./link[@name='{link_name}']/inertial/mass"
            )
            self.assertIsNotNone(mass)
            masses.append(float(mass.attrib["value"]))
        self.assertEqual(masses, [0.015, 0.015])

        gazebo_xacro_path = os.path.join(
            PACKAGE_ROOT,
            "urdf",
            "panda_gz.urdf.xacro",
        )
        with open(gazebo_xacro_path, encoding="utf-8") as xacro_file:
            gazebo_xacro = xacro_file.read()
        self.assertIn("panda_balanced.urdf.xacro", gazebo_xacro)

    def test_vendored_panda_model_has_one_stock_collision_per_finger(self):
        model_path = os.path.join(
            PACKAGE_ROOT,
            "urdf",
            "panda_balanced.urdf.xacro",
        )
        root = ET.parse(model_path).getroot()
        for link_name in ("panda_leftfinger", "panda_rightfinger"):
            link = root.find(f"./link[@name='{link_name}']")
            self.assertIsNotNone(link)
            collisions = link.findall("collision")
            self.assertEqual(len(collisions), 1)
            mesh = collisions[0].find("./geometry/mesh")
            self.assertIsNotNone(mesh)
            self.assertTrue(mesh.attrib["filename"].endswith("/finger.stl"))
        right_origin = root.find(
            "./link[@name='panda_rightfinger']/collision/origin"
        )
        self.assertIsNotNone(right_origin)
        self.assertEqual(right_origin.attrib["rpy"], "0 0 3.14159265359")

    def test_robot_xacro_configures_symmetric_fingertip_contact(self):
        xacro_path = os.path.join(PACKAGE_ROOT, "urdf", "panda_gz.urdf.xacro")
        root = ET.parse(xacro_path).getroot()
        expected = {
            "mu1": "5.0",
            "mu2": "5.0",
            "kp": "1000000.0",
            "kd": "10.0",
            "minDepth": "0.0005",
            "maxVel": "0.02",
        }

        for link_name in ("panda_leftfinger", "panda_rightfinger"):
            surface = root.find(f"./gazebo[@reference='{link_name}']")
            self.assertIsNotNone(surface)
            self.assertEqual(
                {name: surface.findtext(name) for name in expected},
                expected,
            )

    def test_arm_controller_enforces_per_joint_execution_tolerances(self):
        config_path = os.path.join(
            PACKAGE_ROOT,
            "config",
            "gazebo_controllers.yaml",
        )
        with open(config_path, encoding="utf-8") as config_file:
            config_text = config_file.read()
        arm_section = config_text.split("\npanda_arm_controller:\n", 1)[1].split(
            "\npanda_hand_controller:\n",
            1,
        )[0]
        self.assertIn("goal_time: 5.0", arm_section)
        for joint_index in range(1, 8):
            self.assertIn(
                f"panda_joint{joint_index}: {{trajectory: 0.20, goal: 0.01}}",
                arm_section,
            )

    def test_setup_installs_simulation_assets(self):
        setup_path = os.path.join(PACKAGE_ROOT, "setup.py")
        with open(setup_path, encoding="utf-8") as setup_file:
            setup_text = setup_file.read()
        self.assertIn('glob("urdf/*.xacro")', setup_text)
        self.assertIn('glob("worlds/*.sdf")', setup_text)
        self.assertIn('glob("config/*.rviz")', setup_text)
        self.assertIn(
            "gazebo_arm_tracking_test = panda_manipulation.gazebo_arm_tracking_test:main",
            setup_text,
        )

    def test_moveit_controller_targets_gazebo_action(self):
        config_path = os.path.join(
            PACKAGE_ROOT,
            "config",
            "moveit_gazebo_controllers.yaml",
        )
        with open(config_path, encoding="utf-8") as config_file:
            config_text = config_file.read()
        self.assertIn("panda_arm_controller", config_text)
        self.assertIn("action_ns: follow_joint_trajectory", config_text)
        self.assertIn("allowed_execution_duration_scaling: 2.0", config_text)
        self.assertIn("allowed_goal_duration_margin: 6.0", config_text)
        self.assertIn("execution_duration_monitoring: true", config_text)
        for index in range(1, 8):
            self.assertIn(f"panda_joint{index}", config_text)

    def test_hand_controller_commands_both_physical_fingers(self):
        config_path = os.path.join(PACKAGE_ROOT, "config", "gazebo_controllers.yaml")
        with open(config_path, encoding="utf-8") as config_file:
            config_text = config_file.read()
        self.assertIn("panda_hand_controller", config_text)
        self.assertIn("panda_finger_joint1", config_text)
        hand_section = config_text.split("panda_hand_controller:\n", 1)[1]
        self.assertIn("panda_finger_joint2", hand_section)
        self.assertIn("- effort", hand_section)
        self.assertIn("p: 1200.0", hand_section)
        self.assertEqual(hand_section.count("u_clamp_max: 20.0"), 2)
        self.assertEqual(config_text.count("goal: 0.004"), 2)

    def test_physical_pick_uses_contact_limited_grasp(self):
        launch_path = os.path.join(PACKAGE_ROOT, "launch", "gazebo_pick.launch.py")
        with open(launch_path, encoding="utf-8") as launch_file:
            launch_text = launch_file.read()
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        self.assertIn(
            '"gripper_closed_position": gripper_closed_position',
            launch_text,
        )
        self.assertIn('default_value="0.022"', launch_text)
        self.assertIn('"approach_distance": 0.0', launch_text)
        self.assertIn('"pre_grasp_height_offset": 0.08', launch_text)
        self.assertIn('"retreat_height_offset": 0.08', launch_text)
        self.assertIn('"grasp_height_offset": grasp_height_offset', launch_text)
        self.assertIn('"cartesian_to_place"', pipeline_text)
        self.assertIn("payload_transfer_steps", pipeline_text)
        self.assertIn('"place_z": 0.153', launch_text)
        self.assertIn("8 mm table clearance", launch_text)
        self.assertIn('"cartesian_velocity_scaling_factor": 0.12', launch_text)
        self.assertIn(
            '"contact_approach_velocity_scaling_factor": 0.05',
            launch_text,
        )
        self.assertIn('"grasp_preclose_max_target_drift_m",', launch_text)
        self.assertIn('default_value="0.012"', launch_text)
        self.assertIn(
            '"grasp_preclose_max_target_drift_m": (',
            launch_text,
        )
        self.assertIn('"grasp_recovery_max_translation_m": 0.10', launch_text)
        self.assertIn(
            '"pre_grasp_orientation_tolerance": (',
            launch_text,
        )
        self.assertIn('default_value="0.35"', launch_text)
        self.assertIn('"position_tolerance": 0.01', launch_text)
        self.assertIn('"release_settle_s": 2.0', launch_text)
        self.assertIn('"gripper_release_position": 0.028', launch_text)
        self.assertIn('"gripper_release_settle_s": 0.5', launch_text)
        self.assertIn(
            'DeclareLaunchArgument(\n                "enable_tactile_grasp_supervision"',
            launch_text,
        )
        self.assertIn('"enable_tactile_grasp_supervision": ParameterValue(', launch_text)
        self.assertIn('"target_contact_topic": "/gazebo/target_cube/contacts"', launch_text)
        self.assertIn('"tactile_left_finger_token": "panda_leftfinger"', launch_text)
        self.assertIn('"tactile_release_max_retries": 1', launch_text)
        self.assertIn('"release_object_max_horizontal_motion_m": 0.05', launch_text)
        self.assertIn('def evaluate_tactile_grasp(self)', pipeline_text)
        self.assertIn('"tactile_grasp_check"', pipeline_text)
        self.assertIn('def tactile_release_status(self)', pipeline_text)
        self.assertIn('"tactile_release_check"', pipeline_text)
        self.assertIn('def release_object_motion_is_safe(self)', pipeline_text)
        self.assertIn('"release_object_motion_check"', pipeline_text)
        self.assertIn('"pre_place_height_offset": 0.12', launch_text)
        self.assertIn('"pre_place_candidate_attempts": 4', launch_text)
        self.assertIn('"direct_place_candidate_attempts": 4', launch_text)
        self.assertIn('"place_transfer_max_joint_path_length": 4.5', launch_text)
        self.assertIn('"payload_transfer_tolerance_m": 0.06', launch_text)
        self.assertIn(
            '"payload_transfer_velocity_scaling_factor": ParameterValue(',
            launch_text,
        )
        self.assertIn(
            'DeclareLaunchArgument(\n                "ompl_velocity_scaling_factor"',
            launch_text,
        )
        self.assertIn(
            'DeclareLaunchArgument(\n                "payload_transfer_velocity_scaling_factor"',
            launch_text,
        )
        self.assertIn(
            'DeclareLaunchArgument(\n                "place_descent_velocity_scaling_factor"',
            launch_text,
        )
        self.assertIn('default_value="0.03"', launch_text)
        self.assertIn(
            '"place_descent_velocity_scaling_factor": ParameterValue(',
            launch_text,
        )
        self.assertIn(
            '"place_descent_recovery_velocity_scaling_factor": ParameterValue(',
            launch_text,
        )
        self.assertIn('"approach_scene_settle_s": 0.5', launch_text)
        self.assertIn('"preclose_recenter_max_retries": 2', launch_text)
        self.assertIn('"return_home_max_retries": 3', launch_text)
        self.assertIn('"gripper_free_motion_max_retries": 1', launch_text)
        self.assertIn('"ompl_invalid_plan_retries": 2', launch_text)
        self.assertIn('executable="target_object_manager"', launch_text)
        self.assertIn('"manage_collision_object": True', launch_text)
        self.assertIn('DeclareLaunchArgument("target_x"', launch_text)
        self.assertIn('DeclareLaunchArgument("target_y"', launch_text)
        self.assertIn('DeclareLaunchArgument("place_x"', launch_text)
        self.assertIn('DeclareLaunchArgument("place_y"', launch_text)
        self.assertIn(
            'parameters=[{"place_x": place_x, "place_y": place_y}]',
            launch_text,
        )
        self.assertIn('DeclareLaunchArgument("obstacle_x"', launch_text)
        self.assertIn('DeclareLaunchArgument("obstacle_size_z"', launch_text)
        self.assertIn('DeclareLaunchArgument("diagnose_direct_path"', launch_text)
        self.assertIn('DeclareLaunchArgument("enable_dynamic_replanning"', launch_text)
        self.assertIn('"ompl_execution_backend": ompl_execution_backend', launch_text)
        self.assertIn('"require_direct_path_blocked": require_direct_path_blocked', launch_text)
        self.assertIn('DeclareLaunchArgument(\n                "planner_ids_csv"', launch_text)
        self.assertNotIn('"planner_ids": [', launch_text)
        self.assertNotIn('DeclareLaunchArgument("world", default_value="")', launch_text)

    def test_dynamic_replanning_launch_wires_perception_monitor_and_motion(self):
        launch_path = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "dynamic_replanning_demo.launch.py",
        )
        with open(launch_path, encoding="utf-8") as launch_file:
            launch_text = launch_file.read()
        self.assertIn('"ompl_execution_backend": "execute_trajectory"', launch_text)
        self.assertIn('"enable_dynamic_replanning": "true"', launch_text)
        self.assertIn('executable="depth_obstacle_tracker"', launch_text)
        self.assertIn('executable="trajectory_safety_monitor"', launch_text)
        self.assertIn('executable="dynamic_obstacle_controller"', launch_text)

    def test_challenge_diagnostic_runs_before_ompl_pre_grasp(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        prepared = pipeline_text.index("def after_gripper_prepared")
        ompl = pipeline_text.index("def plan_to_pre_grasp", prepared)
        body = pipeline_text[prepared:ompl]
        self.assertIn("self.diagnose_direct_path()", body)
        self.assertIn("challenge scene invalid", body)

    def test_ompl_metrics_are_recorded_from_ompl_result(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        reset_result = pipeline_text.index("def on_reset_result")
        ompl_result = pipeline_text.index("def on_ompl_result", reset_result)
        fixed_home = pipeline_text.index(
            "def plan_fixed_home_pre_grasp_diagnostic",
            ompl_result,
        )
        self.assertNotIn(
            "self.ompl_metrics.append",
            pipeline_text[reset_result:ompl_result],
        )
        self.assertIn(
            "self.ompl_metrics.append",
            pipeline_text[ompl_result:fixed_home],
        )

    def test_invalid_ompl_plan_is_replanned_without_weakening_collision_checks(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        ompl_result = pipeline_text.index("def on_ompl_result")
        fixed_home = pipeline_text.index(
            "def plan_fixed_home_pre_grasp_diagnostic",
            ompl_result,
        )
        body = pipeline_text[ompl_result:fixed_home]
        self.assertIn("error_code == -2", body)
        self.assertIn("self.ompl_invalid_plan_retries", body)
        self.assertIn('f"{step_name}_invalid_plan_replan"', body)
        self.assertIn("self.retry_ompl_step", body)

    def test_physics_benchmark_supports_interactive_visualization(self):
        script_path = os.path.join(
            PACKAGE_ROOT,
            "..",
            "..",
            "scripts",
            "run_gazebo_physics_benchmark.py",
        )
        with open(script_path, encoding="utf-8") as script_file:
            script_text = script_file.read()
        self.assertIn('"--rviz"', script_text)
        self.assertIn('"--gazebo-gui"', script_text)
        self.assertIn('"--hold-after-result-s"', script_text)
        self.assertIn("validation_elapsed_s", script_text)
        self.assertIn('"place_x:=0.50"', script_text)
        self.assertIn('"place_y:=-0.30"', script_text)
        self.assertIn('"ompl_velocity_scaling_factor:=0.05"', script_text)
        self.assertIn(
            '"payload_transfer_velocity_scaling_factor:=0.03"',
            script_text,
        )
        self.assertIn(
            '"place_descent_velocity_scaling_factor:=0.01"',
            script_text,
        )
        self.assertIn(
            '"place_descent_recovery_velocity_scaling_factor:=0.02"',
            script_text,
        )
        self.assertIn(
            '"grasp_preclose_max_target_drift_m:=0.005"',
            script_text,
        )
        self.assertNotIn('"gui:=false"', script_text)
        self.assertNotIn('"rviz:=false"', script_text)
        self.assertIn('"--challenge-scenario"', script_text)
        self.assertIn(
            '"--challenge-scenario requires --challenge-obstacles"',
            script_text,
        )
        self.assertIn("challenge_pool[index % len(challenge_pool)]", script_text)
        self.assertIn('"--resume"', script_text)
        self.assertIn("read_report_rows(args.csv)", script_text)
        self.assertIn("resume CSV does not match the requested matrix", script_text)
        self.assertIn("def isolated_trial_environment", script_text)
        self.assertIn('environment["ROS_DOMAIN_ID"]', script_text)
        self.assertIn('environment["GZ_PARTITION"]', script_text)
        self.assertIn("world_path or DEFAULT_WORLD_PATH", script_text)
        self.assertIn("stop_gazebo_world(effective_world_path)", script_text)

    def test_challenge_uses_ompl_for_obstacle_aware_place_transfer(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        lift = pipeline_text.index("def after_lift")
        recovery = pipeline_text.index("def should_try_approach_recovery", lift)
        body = pipeline_text[lift:recovery]
        self.assertIn("self.use_ompl_for_place", body)
        self.assertIn('"ompl_to_pre_place"', body)
        self.assertIn('"pre_place_pose"', body)
        self.assertIn('"cartesian_place_descent"', body)
        self.assertIn('"cartesian_to_place"', body)

    def test_place_descent_uses_payload_transfer_speed(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        start = pipeline_text.index("payload_transfer_steps =")
        end = pipeline_text.index("if step_name in contact_approach_steps", start)
        body = pipeline_text[start:end]
        self.assertIn('"cartesian_place_descent"', body)

    def test_pre_place_candidate_is_descent_validated_before_execution(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        start = pipeline_text.index("def plan_pre_place_candidate")
        end = pipeline_text.index("def after_pre_place", start)
        body = pipeline_text[start:end]
        self.assertIn("goal.planning_options = self.make_planning_options()", body)
        self.assertIn("start_state_override=end_state", body)
        self.assertIn('"pre_place_descent_validation"', body)
        self.assertIn("self.retry_or_fail_pre_place_candidate", body)
        self.assertIn("self.handle_successful_trajectory", body)
        self.assertIn("path_length > self.place_transfer_max_joint_path_length", body)
        self.assertLess(
            body.index('"pre_place_descent_validation"'),
            body.index("self.handle_successful_trajectory"),
        )

    def test_direct_place_fallback_prevalidates_retreat_and_payload(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        direct_start = pipeline_text.index("def plan_direct_place_candidate")
        approach_recovery = pipeline_text.index(
            "def should_try_approach_recovery",
            direct_start,
        )
        body = pipeline_text[direct_start:approach_recovery]
        self.assertIn("goal.planning_options = self.make_planning_options()", body)
        self.assertIn('self.candidate["pre_place_pose"]', body)
        self.assertIn('"direct_place_retreat_validation"', body)
        self.assertIn('"physical_transfer_check"', body)
        self.assertIn('self.verify_payload_after_transfer("pre_place_pose")', body)
        self.assertIn('self.verify_payload_after_transfer("place_pose")', body)
        self.assertIn("self.payload_transfer_velocity_scaling_factor", body)

        cartesian_start = pipeline_text.index("def make_cartesian_request")
        cartesian_end = pipeline_text.index("def measured_robot_state", cartesian_start)
        cartesian_body = pipeline_text[cartesian_start:cartesian_end]
        self.assertIn("place_descent_steps", cartesian_body)
        self.assertIn("self.place_descent_velocity_scaling_factor", cartesian_body)
        self.assertIn(
            "self.place_descent_recovery_velocity_scaling_factor",
            cartesian_body,
        )

    def test_payload_cartesian_steps_require_nearly_complete_paths(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        start = pipeline_text.index("def required_cartesian_fraction")
        end = pipeline_text.index("def after_lift", start)
        body = pipeline_text[start:end]
        self.assertIn('"cartesian_pre_grasp_alignment"', body)
        self.assertIn('"cartesian_approach"', body)
        self.assertIn('"cartesian_place_descent"', body)
        self.assertIn('"cartesian_retreat"', body)
        self.assertIn("max(self.cartesian_min_fraction, 0.98)", body)
        self.assertIn("def trajectory_timing_error", body)

    def test_cartesian_execution_rejects_non_monotonic_timestamps(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        start = pipeline_text.index("def on_cartesian_result")
        end = pipeline_text.index("def after_lift", start)
        body = pipeline_text[start:end]
        timing_check = body.index("self.trajectory_timing_error(response.solution)")
        execution = body.index("self.handle_successful_trajectory")
        self.assertLess(timing_check, execution)
        self.assertIn("invalid trajectory timing", body)
        self.assertIn("self.should_try_pre_grasp_alignment_recovery(step_name)", body)

    def test_release_retreat_has_collision_aware_ompl_fallback(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        cartesian_result = pipeline_text.index("def on_cartesian_result")
        lift = pipeline_text.index("def after_lift", cartesian_result)
        body = pipeline_text[cartesian_result:lift]
        self.assertIn('step_name == "cartesian_retreat"', body)
        self.assertIn("self.allow_ompl_release_retreat_fallback", body)
        self.assertIn("self.plan_bounded_release_retreat", body)
        bounded_start = pipeline_text.index("def plan_bounded_release_retreat")
        bounded_end = pipeline_text.index("def after_return_home_scene_settle", bounded_start)
        bounded_body = pipeline_text[bounded_start:bounded_end]
        self.assertIn('"ompl_retreat_fallback"', bounded_body)
        self.assertIn("goal.planning_options = self.make_planning_options()", bounded_body)
        self.assertIn(
            "path_length > self.release_retreat_max_joint_path_length", bounded_body
        )
        self.assertIn("within safe limit", bounded_body)

        manager_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "target_object_manager.py",
        )
        with open(manager_path, encoding="utf-8") as manager_file:
            manager_text = manager_file.read()
        launch_path = os.path.join(PACKAGE_ROOT, "launch", "gazebo_pick.launch.py")
        with open(launch_path, encoding="utf-8") as launch_file:
            launch_text = launch_file.read()
        self.assertIn(
            '{"cartesian_retreat", "ompl_retreat_fallback"}',
            manager_text,
        )
        self.assertIn('step == "object_collision_cleared"', manager_text)
        self.assertIn('"object_collision_cleared"', pipeline_text)
        self.assertIn('self.declare_parameter("use_live_release_pose", False)', manager_text)
        self.assertIn("self.use_live_release_pose", manager_text)
        self.assertIn('"use_live_release_pose": ParameterValue(', launch_text)

    def test_physical_pick_opens_gripper_before_pre_grasp_motion(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        start = pipeline_text.index("def start_pipeline")
        prepare = pipeline_text.index("def prepare_gripper_for_approach", start)
        plan = pipeline_text.index("def plan_to_pre_grasp", prepare)
        prepare_body = pipeline_text[prepare:plan]
        self.assertIn('"gripper_open"', prepare_body)
        self.assertIn("self.plan_to_pre_grasp", prepare_body)

    def test_execution_waits_for_arm_controller_action(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        self.assertIn(
            '"/panda_arm_controller/follow_joint_trajectory"',
            pipeline_text,
        )
        readiness_start = pipeline_text.index("def on_grasp_candidates")
        pipeline_start = pipeline_text.index("def start_pipeline", readiness_start)
        readiness_body = pipeline_text[readiness_start:pipeline_start]
        self.assertIn("self.arm_trajectory_client.wait_for_server", readiness_body)

    def test_executed_cartesian_segments_use_measured_start_state(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        request_start = pipeline_text.index("def make_cartesian_request")
        measured_start = pipeline_text.index("def measured_robot_state", request_start)
        request_body = pipeline_text[request_start:measured_start]
        self.assertIn("measured_start_state = self.measured_robot_state()", request_body)
        self.assertIn("request.start_state = measured_start_state", request_body)
        result_start = pipeline_text.index("def on_cartesian_result")
        result_end = pipeline_text.index("def retry_cartesian_after_timing_error", result_start)
        result_body = pipeline_text[result_start:result_end]
        self.assertIn("self.execution_start_check(response.solution)", result_body)
        self.assertIn("cartesian_start_state_replan_counts", result_body)
        self.assertIn("discarding the trajectory and replanning", result_body)
        self.assertLess(
            result_body.index("self.execution_start_check(response.solution)"),
            result_body.index("self.handle_successful_trajectory"),
        )

    def test_payload_transfer_execution_replan_is_bounded_and_supervised(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        validation_start = pipeline_text.index(
            "def on_pre_place_descent_validation"
        )
        candidate_retry = pipeline_text.index(
            "def retry_or_fail_pre_place_candidate",
            validation_start,
        )
        validation_body = pipeline_text[validation_start:candidate_retry]
        self.assertIn("self.plan_pre_place_candidate", validation_body)
        check_start = pipeline_text.index(
            "def should_replan_payload_transfer_execution"
        )
        schedule_start = pipeline_text.index(
            "def schedule_payload_transfer_execution_replan",
            check_start,
        )
        check_body = pipeline_text[check_start:schedule_start]
        self.assertIn('"ompl_to_pre_place"', check_body)
        self.assertIn('"cartesian_to_pre_place"', check_body)
        self.assertIn('"cartesian_place_descent_stage_"', check_body)
        self.assertIn(
            "self.payload_transfer_execution_replan_max_retries",
            check_body,
        )
        self.assertIn("self.tactile_contact_status()", check_body)
        self.assertIn("self.verify_payload_after_transfer", check_body)

    def test_payload_descent_endpoint_acceptance_is_strictly_supervised(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        check_start = pipeline_text.index(
            "def payload_endpoint_outcome_after_failure"
        )
        fallback_start = pipeline_text.index(
            "def try_pre_place_transfer_ompl_fallback",
            check_start,
        )
        check_body = pipeline_text[check_start:fallback_start]
        self.assertIn('"cartesian_place_descent_stage_"', check_body)
        self.assertIn("normalized_joint_limit_margin", check_body)
        self.assertIn("self.tf_buffer.lookup_transform", check_body)
        self.assertIn("quaternion_angular_distance", check_body)
        self.assertIn("payload_endpoint_task_space_check", check_body)
        self.assertIn("self.tactile_contact_status()", check_body)
        self.assertIn("self.verify_payload_after_transfer", check_body)
        self.assertIn(
            "self.place_descent_stage_payload_tolerance_m",
            check_body,
        )
        self.assertIn(
            "self.place_descent_endpoint_orientation_tolerance_rad",
            check_body,
        )

    def test_preclose_recenter_has_an_independent_retry_budget(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        start = pipeline_text.index("def after_grasp(self)")
        end = pipeline_text.index("def after_gripper_closed", start)
        body = pipeline_text[start:end]
        self.assertIn("self.preclose_recenter_retry_count", body)
        self.assertIn("self.preclose_recenter_max_retries", body)
        self.assertNotIn("self.grasp_probe_retry_count += 1", body)

    def test_execution_failure_preserves_terminal_payload_diagnostic(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT, "panda_manipulation", "pick_plan_pipeline.py"
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        start = pipeline_text.index("def on_execute_result(")
        end = pipeline_text.index("def arm_endpoint_outcome_after_failure", start)
        body = pipeline_text[start:end]
        self.assertIn(
            "if not self.active:\n                    return\n"
            "                payload_endpoint_outcome =",
            body,
        )
        self.assertIn(
            "if not self.active:\n                    return\n"
            "                if payload_endpoint_outcome is not None:",
            body,
        )

    def test_pre_grasp_cartesian_execution_replan_is_bounded(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        check_start = pipeline_text.index(
            "def should_replan_cartesian_execution"
        )
        schedule_start = pipeline_text.index(
            "def schedule_cartesian_execution_replan",
            check_start,
        )
        payload_start = pipeline_text.index(
            "def should_replan_payload_transfer_execution",
            schedule_start,
        )
        check_body = pipeline_text[check_start:schedule_start]
        schedule_body = pipeline_text[schedule_start:payload_start]
        self.assertIn('"cartesian_pre_grasp_alignment"', check_body)
        self.assertIn('"cartesian_approach"', check_body)
        self.assertIn('"cartesian_grasp_recovery_reapproach"', check_body)
        self.assertIn("error_code == -4", check_body)
        self.assertIn(
            "self.cartesian_execution_replan_max_retries",
            check_body,
        )
        self.assertIn("self.current_start_state = None", pipeline_text)
        self.assertIn("self.create_timer", schedule_body)

    def test_prevalidated_pre_grasp_trajectory_is_the_executed_trajectory(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        select_start = pipeline_text.index(
            "def select_prevalidated_grasp_candidate"
        )
        plan_start = pipeline_text.index("def plan_to_pre_grasp", select_start)
        plan_end = pipeline_text.index(
            "def requires_joint_state_before_start",
            plan_start,
        )
        select_body = pipeline_text[select_start:plan_start]
        plan_body = pipeline_text[plan_start:plan_end]
        self.assertIn("selected.get(\"pre_grasp_trajectory\")", select_body)
        self.assertIn("prevalidated_pre_grasp_trajectory", plan_body)
        self.assertIn("self.handle_successful_trajectory", plan_body)
        self.assertIn("prevalidated_trajectory", plan_body)
        self.assertLess(
            plan_body.index("self.handle_successful_trajectory"),
            plan_body.index("self.plan_ompl_step"),
        )

    def test_joint_limit_candidate_prevalidation_resampling_is_bounded(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        select_start = pipeline_text.index(
            "def select_prevalidated_grasp_candidate"
        )
        resample_start = pipeline_text.index(
            "def resample_grasp_candidate_prevalidation",
            select_start,
        )
        next_start = pipeline_text.index("def after_gripper_prepared", resample_start)
        select_body = pipeline_text[select_start:resample_start]
        resample_body = pipeline_text[resample_start:next_start]
        self.assertIn("grasp_candidate_prevalidation_max_rounds", select_body)
        self.assertIn("minimum joint-limit margin", select_body)
        self.assertIn("grasp_candidate_prevalidation_round += 1", resample_body)
        self.assertIn("self.schedule_grasp_candidate_prevalidation", resample_body)

    def test_target_collision_is_removed_only_for_contact_approach(self):
        manager_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "target_object_manager.py",
        )
        with open(manager_path, encoding="utf-8") as manager_file:
            manager_text = manager_file.read()
        self.assertIn('if step == "cartesian_pre_grasp_alignment"', manager_text)
        self.assertIn("self.publish_removed_from_world()", manager_text)
        self.assertIn("elif self.approaching:", manager_text)
        self.assertIn(
            'elif step in {"cartesian_retreat", "ompl_retreat_fallback"} '
            "and self.release_pending",
            manager_text,
        )

    def test_release_removes_attached_and_world_collision_until_retreat_finishes(self):
        manager_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "target_object_manager.py",
        )
        with open(manager_path, encoding="utf-8") as manager_file:
            manager_text = manager_file.read()
        detach_start = manager_text.index("def publish_detached(self)")
        restore_start = manager_text.index("def publish_detached_at_place", detach_start)
        detach_body = manager_text[detach_start:restore_start]
        self.assertIn("self.publisher.publish(detach_scene)", detach_body)
        self.assertIn("self.publisher.publish(remove_scene)", detach_body)
        self.assertLess(
            detach_body.index("self.publisher.publish(detach_scene)"),
            detach_body.index("self.publisher.publish(remove_scene)"),
        )
        self.assertIn(
            "remove_scene.world.collision_objects.append(remove_world)", detach_body
        )
        self.assertIn(
            "detach_scene.robot_state.attached_collision_objects.append(detach)",
            detach_body,
        )
        self.assertIn("remove_world.operation = CollisionObject.REMOVE", detach_body)
        self.assertIn('self.object_pose_from_hand_waypoint("place_pose")', manager_text)

    def test_pre_grasp_alignment_recovers_via_elevated_ompl_staging(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        cartesian_result = pipeline_text.index("def on_cartesian_result")
        approach_recovery = pipeline_text.index("def plan_approach_recovery", cartesian_result)
        body = pipeline_text[cartesian_result:approach_recovery]
        self.assertIn("self.should_try_pre_grasp_alignment_recovery(step_name)", body)
        self.assertIn('step_name == "cartesian_pre_grasp_alignment"', body)
        self.assertIn('"ompl_to_pre_grasp_alignment_staging"', body)
        self.assertIn('"pre_grasp_alignment_staging_pose"', body)
        self.assertIn('"ompl_to_strict_pre_grasp"', body)
        self.assertIn("def after_strict_pre_grasp", body)
        self.assertIn("constrain_orientation=True", body)
        self.assertIn(
            'self.cartesian_retry_used.pop("cartesian_pre_grasp_alignment", None)',
            body,
        )
        self.assertIn(
            'step_name == "cartesian_pre_grasp_alignment"',
            body,
        )
        self.assertIn(
            "self.pre_grasp_alignment_recovery_min_fraction",
            body,
        )
        self.assertIn("0.95", body)

    def test_approach_recovery_preserves_grasp_orientation(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            body = pipeline_file.read()
        start = body.index("def plan_approach_recovery")
        end = body.index("def after_approach_recovery", start)
        recovery_body = body[start:end]
        self.assertIn('"ompl_to_approach_recovery"', recovery_body)
        self.assertIn("constrain_orientation=True", recovery_body)
        self.assertIn(
            "planner_id_override=self.recovery_planner_id",
            recovery_body,
        )
        self.assertIn(
            "orientation_tolerance=orientation_tolerance",
            recovery_body,
        )
        self.assertIn("self.recovery_orientation_tolerance_step", recovery_body)
        self.assertIn("self.approach_recovery_ratios", recovery_body)
        self.assertIn(
            'self.cartesian_retry_used.pop("cartesian_approach", None)',
            body,
        )

    def test_approach_recovery_tries_next_candidate_after_ompl_failure(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            body = pipeline_file.read()
        start = body.index("def on_ompl_result")
        end = body.index("def plan_fixed_home_pre_grasp_diagnostic", start)
        result_body = body[start:end]
        self.assertIn('step_name == "ompl_to_approach_recovery"', result_body)
        self.assertIn("self.has_approach_recovery_candidate()", result_body)
        self.assertIn("self.plan_approach_recovery()", result_body)
        self.assertIn("self.recovery_ompl_retry_counts", result_body)
        self.assertIn("error_code == 99999", result_body)

    def test_equivalent_grasp_orientation_is_bounded_and_scene_synchronized(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        manager_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "target_object_manager.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_body = pipeline_file.read()
        with open(manager_path, encoding="utf-8") as manager_file:
            manager_body = manager_file.read()
        start = pipeline_body.index("def plan_equivalent_grasp_orientation")
        end = pipeline_body.index("def should_try_approach_recovery", start)
        recovery_body = pipeline_body[start:end]
        self.assertIn(
            "self.equivalent_grasp_recovery_yaw_offsets_deg",
            recovery_body,
        )
        self.assertIn("equivalent_grasp_candidate(", recovery_body)
        self.assertIn('"ompl_to_equivalent_grasp_staging"', recovery_body)
        self.assertIn('"ompl_to_equivalent_grasp_pre_grasp"', recovery_body)
        self.assertIn("constrain_orientation=True", recovery_body)
        self.assertIn('"equivalent_grasp_pre_grasp_ready"', recovery_body)
        self.assertIn("self.after_pre_grasp_alignment()", recovery_body)
        self.assertIn("self.publish_active_candidate()", recovery_body)
        self.assertIn('"/active_grasp_candidate"', pipeline_body)
        self.assertIn('"/active_grasp_candidate"', manager_body)
        self.assertIn("def on_active_grasp_candidate", manager_body)

    def test_grasp_candidates_are_prevalidated_before_any_execution(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        launch_path = os.path.join(PACKAGE_ROOT, "launch", "gazebo_pick.launch.py")
        dynamic_launch_path = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "dynamic_replanning_demo.launch.py",
        )
        manager_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "target_object_manager.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_body = pipeline_file.read()
        with open(launch_path, encoding="utf-8") as launch_file:
            launch_body = launch_file.read()
        with open(dynamic_launch_path, encoding="utf-8") as launch_file:
            dynamic_launch_body = launch_file.read()
        with open(manager_path, encoding="utf-8") as manager_file:
            manager_body = manager_file.read()

        start = pipeline_body.index("def begin_grasp_candidate_prevalidation")
        end = pipeline_body.index("def after_gripper_prepared", start)
        prevalidation_body = pipeline_body[start:end]
        self.assertIn("goal.planning_options = self.make_planning_options()", prevalidation_body)
        self.assertIn("request.avoid_collisions = True", prevalidation_body)
        self.assertIn(
            '"grasp_candidate_prevalidation_contact"',
            prevalidation_body,
        )
        self.assertIn(
            '"grasp_candidate_prevalidation_restore"',
            prevalidation_body,
        )
        self.assertIn("self.grasp_candidate_prevalidation_min_fraction", prevalidation_body)
        self.assertIn("normalized_joint_limit_margin(", prevalidation_body)
        self.assertIn("grasp_candidate_prevalidation_score(", prevalidation_body)
        self.assertIn(
            "callback or self.prepare_gripper_for_approach",
            prevalidation_body,
        )
        self.assertIn(
            'DeclareLaunchArgument(\n                "enable_grasp_candidate_prevalidation"',
            launch_body,
        )
        self.assertIn(
            '"enable_grasp_candidate_prevalidation",\n                default_value="false"',
            dynamic_launch_body,
        )
        self.assertIn(
            '"defer_grasp_candidate_prevalidation_until_dynamic_replan",\n                default_value="true"',
            dynamic_launch_body,
        )
        self.assertIn(
            "self.prevalidate_grasp_candidates_after_dynamic_replan",
            pipeline_body,
        )
        self.assertIn(
            "self.plan_prevalidated_dynamic_grasp",
            pipeline_body,
        )
        self.assertIn(
            "planner_id_override=self.recovery_planner_id",
            pipeline_body,
        )
        self.assertIn(
            "def grasp_prevalidation_planner_id",
            pipeline_body,
        )
        self.assertIn(
            'step == "grasp_candidate_prevalidation_contact"',
            manager_body,
        )

    def test_pre_grasp_alignment_executes_only_checked_prefixes(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            body = pipeline_file.read()
        start = body.index("def try_execute_pre_grasp_alignment_prefix")
        end = body.index("def required_cartesian_fraction", start)
        prefix_body = body[start:end]
        self.assertIn("self.pre_grasp_alignment_recovery_used", prefix_body)
        self.assertIn("int(response.error_code.val) != 1", prefix_body)
        self.assertIn("self.trajectory_timing_error(response.solution)", prefix_body)
        self.assertIn('"cartesian_pre_grasp_alignment_prefix"', prefix_body)
        self.assertIn("self.after_pre_grasp_alignment_prefix", prefix_body)

    def test_invalid_cartesian_timing_uses_bounded_moveit_replans(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            body = pipeline_file.read()
        result_start = body.index("def on_cartesian_result")
        retry_start = body.index(
            "def retry_cartesian_after_timing_error",
            result_start,
        )
        retry_end = body.index(
            "def try_execute_pre_grasp_alignment_prefix",
            retry_start,
        )
        result_body = body[result_start:retry_start]
        retry_body = body[retry_start:retry_end]
        self.assertIn("self.retry_cartesian_after_timing_error(", result_body)
        self.assertIn("self.cartesian_timing_retry_max_steps", retry_body)
        self.assertIn("self.make_cartesian_request(", retry_body)
        self.assertIn("step_name=step_name", retry_body)
        self.assertIn("self.on_cartesian_result(", retry_body)
        self.assertNotIn("time_from_start =", retry_body)

    def test_contact_prefix_recovery_preserves_collision_and_timing_checks(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            body = pipeline_file.read()
        start = body.index("def try_execute_contact_prefix")
        end = body.index("def after_pre_grasp_alignment_prefix", start)
        prefix_body = body[start:end]
        self.assertIn('"cartesian_approach"', prefix_body)
        self.assertIn('"cartesian_grasp_probe"', prefix_body)
        self.assertIn("int(response.error_code.val) != 1", prefix_body)
        self.assertIn("self.contact_prefix_min_fraction", prefix_body)
        self.assertIn("self.trajectory_timing_error(response.solution)", prefix_body)
        self.assertIn("self.after_contact_prefix(", prefix_body)
        self.assertIn("self.cartesian_retry_used.pop(step_name, None)", prefix_body)
        self.assertIn("self.plan_cartesian_step(step_name, pose_key", prefix_body)

    def test_gripper_goal_rejection_has_activation_retry(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            body = pipeline_file.read()
        self.assertIn('"gripper_goal_rejection_max_retries"', body)
        self.assertIn("schedule_gripper_trajectory_retry", body)
        self.assertIn("gripper_goal_rejection_retry_total", body)

    def test_gazebo_release_uses_current_orientation_then_returns_home(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        release = pipeline_text.index("def after_release_settle")
        failure = pipeline_text.index("def finish_failed", release)
        body = pipeline_text[release:failure]
        self.assertIn("self.current_release_clearance_pose()", body)
        self.assertIn('self.candidate.get("pre_place_pose")', body)
        self.assertIn('"release_clearance_pose"', body)
        self.assertIn("self.tf_buffer.lookup_transform", body)
        self.assertIn("float(rotation.w)", body)
        self.assertIn("def plan_return_home", body)
        self.assertIn('"return_home"', body)

        staged_release = pipeline_text.index("def finish_success")
        release_open = pipeline_text.index("def after_release_gripper_open", staged_release)
        staged_body = pipeline_text[staged_release:release_open]
        self.assertIn('"gripper_release"', staged_body)
        self.assertIn("self.gripper_release_position", staged_body)
        self.assertIn("self.gripper_release_settle_s", staged_body)
        self.assertIn("def complete_release_open", staged_body)
        self.assertIn("self.release_reference_target_pose", staged_body)
        self.assertIn("self.after_return_home", body)

        launch_path = os.path.join(PACKAGE_ROOT, "launch", "gazebo_pick.launch.py")
        with open(launch_path, encoding="utf-8") as launch_file:
            launch_text = launch_file.read()
        self.assertIn('"allow_ompl_release_retreat_fallback": True', launch_text)
        self.assertIn('"release_retreat_max_joint_path_length": 1.5', launch_text)
        self.assertIn('"return_home_after_success": True', launch_text)
        self.assertIn('"return_home_max_retries": 3', launch_text)
        self.assertIn('"return_home_scene_settle_s": 1.0', launch_text)
        self.assertIn(
            '"release_object_max_vertical_motion_m": 0.06',
            launch_text,
        )
        self.assertIn('"place_object_clearance_m": 0.008', launch_text)
        self.assertIn('"place_feedback_tolerance_m": 0.05', launch_text)
        self.assertIn('"place_feedback_max_retries": 1', launch_text)
        self.assertIn(
            'place_descent_stage_count = LaunchConfiguration("place_descent_stage_count")',
            launch_text,
        )
        self.assertIn('default_value="6"', launch_text)
        self.assertIn(
            '"place_descent_stage_count": ParameterValue(',
            launch_text,
        )
        self.assertIn(
            '"place_descent_stage_max_horizontal_motion_m": 0.01',
            launch_text,
        )
        self.assertIn(
            '"place_descent_stage_payload_tolerance_m": 0.015',
            launch_text,
        )
        self.assertIn("def capture_place_descent_payload_anchor", pipeline_text)
        self.assertIn("place_descent_object_in_hand_pose", pipeline_text)
        self.assertIn("servo_watchdog_abort_stop_monotonic_ns", pipeline_text)
        self.assertIn("place_descent_payload_reanchor", pipeline_text)
        self.assertIn(
            '"release_clearance_height": release_clearance_height',
            launch_text,
        )

    def test_return_home_replans_only_plan_only_failures(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        start = pipeline_text.index("def plan_return_home")
        end = pipeline_text.index("def after_return_home", start)
        body = pipeline_text[start:end]
        self.assertIn("goal.planning_options = self.make_planning_options()", body)
        self.assertIn("self.retry_or_fail_return_home", body)
        self.assertIn(
            "self.return_home_retry_count < self.return_home_max_retries",
            body,
        )
        self.assertIn('self.record_step("return_home_replan", False', body)
        self.assertIn("self.plan_return_home()", body)

    def test_pre_grasp_is_cartesian_aligned_before_contact_approach(self):
        pipeline_path = os.path.join(
            PACKAGE_ROOT,
            "panda_manipulation",
            "pick_plan_pipeline.py",
        )
        with open(pipeline_path, encoding="utf-8") as pipeline_file:
            pipeline_text = pipeline_file.read()
        pre_grasp = pipeline_text.index("def after_pre_grasp")
        alignment = pipeline_text.index("def after_pre_grasp_alignment", pre_grasp)
        pre_grasp_body = pipeline_text[pre_grasp:alignment]
        self.assertIn('"cartesian_pre_grasp_alignment"', pre_grasp_body)
        self.assertIn('"pre_grasp_pose"', pre_grasp_body)

        world_path = os.path.join(PACKAGE_ROOT, "worlds", "panda_table.sdf")
        with open(world_path, encoding="utf-8") as world_file:
            world_text = world_file.read()
        self.assertIn("<mu>2.0</mu><mu2>2.0</mu2>", world_text)

    def test_bridge_exposes_target_pose(self):
        config_path = os.path.join(PACKAGE_ROOT, "config", "gazebo_bridge.yaml")
        with open(config_path, encoding="utf-8") as config_file:
            config_text = config_file.read()
        self.assertIn("/model/target_cube/pose", config_text)
        self.assertIn("/gazebo/target_pose", config_text)
        self.assertIn("geometry_msgs/msg/Pose", config_text)

    def test_world_contains_overhead_camera_and_aruco_marker(self):
        world_path = os.path.join(PACKAGE_ROOT, "worlds", "panda_table.sdf")
        root = ET.parse(world_path).getroot()
        sensor = root.find(
            ".//model[@name='overhead_camera']//sensor[@name='rgb_camera']"
        )
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.attrib["type"], "camera")
        self.assertEqual(sensor.find("topic").text, "/camera/image_raw")
        self.assertEqual(
            sensor.find("camera/camera_info_topic").text,
            "/camera/camera_info",
        )
        target = root.find(".//model[@name='target_cube']/link")
        visual_names = {
            visual.attrib["name"] for visual in target.findall("visual")
        }
        self.assertIn("aruco_black_plate", visual_names)
        self.assertEqual(
            len(
                [
                    name
                    for name in visual_names
                    if name.startswith("aruco_white_")
                ]
            ),
            8,
        )

    def test_bridge_exposes_camera_image_and_intrinsics(self):
        config_path = os.path.join(PACKAGE_ROOT, "config", "gazebo_bridge.yaml")
        with open(config_path, encoding="utf-8") as config_file:
            config_text = config_file.read()
        self.assertIn("sensor_msgs/msg/Image", config_text)
        self.assertIn("sensor_msgs/msg/CameraInfo", config_text)
        self.assertIn("/camera/image_raw", config_text)
        self.assertIn("/camera/camera_info", config_text)

    def test_gazebo_pick_can_switch_from_synthetic_to_camera_perception(self):
        launch_path = os.path.join(
            PACKAGE_ROOT,
            "launch",
            "gazebo_pick.launch.py",
        )
        with open(launch_path, encoding="utf-8") as launch_file:
            launch_text = launch_file.read()
        self.assertIn('DeclareLaunchArgument(\n                "use_camera_perception"', launch_text)
        self.assertIn('"use_rgbd_target_perception"', launch_text)
        self.assertIn("PythonExpression", launch_text)
        self.assertIn("use_rgbd_target_perception,", launch_text)
        self.assertIn("IfCondition(use_camera_perception)", launch_text)
        self.assertIn(
            'DeclareLaunchArgument("marker_size_m", default_value="0.044")',
            launch_text,
        )
        self.assertIn('default_value="-0.04"', launch_text)
        self.assertIn('"marker_size_m": marker_size_m', launch_text)
        self.assertIn(
            '"object_center_offset_z_m": object_center_offset_z_m',
            launch_text,
        )


if __name__ == "__main__":
    unittest.main()
