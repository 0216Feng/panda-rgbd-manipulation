from glob import glob
from setuptools import setup

package_name = "panda_manipulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (
            f"share/{package_name}/config",
            glob("config/*.yaml")
            + glob("config/*.rviz")
            + glob("config/*.config"),
        ),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
        (f"share/{package_name}/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HUO ZHIFENG",
    maintainer_email="90129509+0216Feng@users.noreply.github.com",
    description="Panda manipulation planning, perception, task management, and benchmarking.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scene_manager = panda_manipulation.scene_manager:main",
            "servo_readiness_test = panda_manipulation.servo_readiness:main",
            "aruco_pose_estimator = panda_manipulation.aruco_pose_estimator:main",
            "aruco_perception_validator = panda_manipulation.aruco_perception_validator:main",
            "rgbd_target_pose_estimator = panda_manipulation.rgbd_target_pose_estimator:main",
            "cartesian_approach_planner = panda_manipulation.cartesian_approach_planner:main",
            "depth_obstacle_tracker = panda_manipulation.depth_obstacle_tracker:main",
            "dynamic_obstacle_controller = panda_manipulation.dynamic_obstacle_controller:main",
            "dynamic_obstacle_perception_validator = panda_manipulation.dynamic_obstacle_perception_validator:main",
            "gazebo_ready_pose = panda_manipulation.gazebo_ready_pose:main",
            "gazebo_gripper_smoke_test = panda_manipulation.gazebo_gripper_smoke_test:main",
            "gazebo_arm_tracking_test = panda_manipulation.gazebo_arm_tracking_test:main",
            "gazebo_pick_validator = panda_manipulation.gazebo_pick_validator:main",
            "grasp_pose_generator = panda_manipulation.grasp_pose_generator:main",
            "manipulation_marker_publisher = panda_manipulation.manipulation_marker_publisher:main",
            "manipulation_task_manager = panda_manipulation.manipulation_task_manager:main",
            "moveit_plan_only_adapter = panda_manipulation.moveit_plan_only_adapter:main",
            "moveit_gazebo_smoke_test = panda_manipulation.moveit_gazebo_smoke_test:main",
            "pick_plan_benchmark_recorder = panda_manipulation.pick_plan_benchmark_recorder:main",
            "pick_plan_pipeline = panda_manipulation.pick_plan_pipeline:main",
            "planner_benchmark_runner = panda_manipulation.planner_benchmark_runner:main",
            "physical_disturbance_injector = panda_manipulation.physical_disturbance_injector:main",
            "synthetic_target_sequence_publisher = panda_manipulation.synthetic_target_sequence_publisher:main",
            "target_object_manager = panda_manipulation.target_object_manager:main",
            "trajectory_safety_monitor = panda_manipulation.trajectory_safety_monitor:main",
            "transfer_diagnostics_recorder = panda_manipulation.transfer_diagnostics:main",
        ],
    },
)
