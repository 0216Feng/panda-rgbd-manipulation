import importlib
import os
import sys
import unittest

PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PACKAGE_ROOT)


class ImportTests(unittest.TestCase):
    def test_ros_nodes_import_without_ros2(self):
        modules = [
            "panda_manipulation.aruco_pose_estimator",
            "panda_manipulation.aruco_perception_validator",
            "panda_manipulation.rgbd_target_pose_estimator",
            "panda_manipulation.cartesian_approach_planner",
            "panda_manipulation.depth_obstacle_tracker",
            "panda_manipulation.dynamic_obstacle_controller",
            "panda_manipulation.dynamic_obstacle_perception_validator",
            "panda_manipulation.gazebo_ready_pose",
            "panda_manipulation.gazebo_gripper_smoke_test",
            "panda_manipulation.gazebo_pick_validator",
            "panda_manipulation.grasp_pose_generator",
            "panda_manipulation.manipulation_marker_publisher",
            "panda_manipulation.manipulation_task_manager",
            "panda_manipulation.moveit_plan_only_adapter",
            "panda_manipulation.moveit_gazebo_smoke_test",
            "panda_manipulation.pick_plan_benchmark_recorder",
            "panda_manipulation.pick_plan_pipeline",
            "panda_manipulation.physical_disturbance_injector",
            "panda_manipulation.planner_benchmark_runner",
            "panda_manipulation.scene_manager",
            "panda_manipulation.synthetic_target_sequence_publisher",
            "panda_manipulation.target_object_manager",
            "panda_manipulation.trajectory_safety_monitor",
            "panda_manipulation.transfer_diagnostics",
            "panda_manipulation.robot_description",
        ]
        for module_name in modules:
            with self.subTest(module=module_name):
                importlib.import_module(module_name)


if __name__ == "__main__":
    unittest.main()
