from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from panda_manipulation.robot_description import build_gazebo_robot_description


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    gui_config = LaunchConfiguration("gui_config")
    world = LaunchConfiguration("world")
    move_to_ready = LaunchConfiguration("move_to_ready")
    run_gripper_test = LaunchConfiguration("run_gripper_test")

    package_share = get_package_share_directory("panda_manipulation")
    default_world = PathJoinSubstitution(
        [FindPackageShare("panda_manipulation"), "worlds", "panda_table.sdf"]
    )
    default_gui_config = PathJoinSubstitution(
        [FindPackageShare("panda_manipulation"), "config", "gazebo_demo.gui.config"]
    )
    robot_description = build_gazebo_robot_description(
        f"{package_share}/urdf/panda_gz.urdf.xacro"
    )

    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("gui_config", default_value=default_gui_config),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("move_to_ready", default_value="true"),
            DeclareLaunchArgument("run_gripper_test", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={
                    "gz_args": [
                        "-r -v 3 --gui-config ",
                        gui_config,
                        " ",
                        world,
                    ]
                }.items(),
                condition=IfCondition(gui),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={"gz_args": ["-r -s -v 3 ", world]}.items(),
                condition=UnlessCondition(gui),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="overhead_camera_static_tf",
                output="screen",
                arguments=[
                    "--x",
                    "0.65",
                    "--y",
                    "0.0",
                    "--z",
                    "0.93",
                    "--roll",
                    "3.14159265359",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "-1.57079632679",
                    "--frame-id",
                    "panda_link0",
                    "--child-frame-id",
                    "overhead_camera_optical_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="overhead_depth_camera_static_tf",
                output="screen",
                arguments=[
                    "--x",
                    "0.65",
                    "--y",
                    "0.0",
                    "--z",
                    "0.93",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "1.57079632679",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "panda_link0",
                    "--child-frame-id",
                    "overhead_depth_camera_frame",
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gazebo_bridge",
                output="screen",
                parameters=[
                    {
                        "config_file": (
                            f"{package_share}/config/gazebo_bridge.yaml"
                        )
                    }
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gazebo_set_pose_bridge",
                output="screen",
                arguments=[
                    "/world/panda_table/set_pose@ros_gz_interfaces/srv/SetEntityPose"
                ],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_panda",
                output="screen",
                arguments=[
                    "-world",
                    "panda_table",
                    "-topic",
                    "/robot_description",
                    "-name",
                    "panda",
                    "-allow_renaming",
                    "false",
                ],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "joint_state_broadcaster",
                            "--controller-manager",
                            "/controller_manager",
                            "--controller-manager-timeout",
                            "30",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "panda_arm_controller",
                            "--controller-manager",
                            "/controller_manager",
                            "--controller-manager-timeout",
                            "30",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "panda_hand_controller",
                            "--controller-manager",
                            "/controller_manager",
                            "--controller-manager-timeout",
                            "30",
                        ],
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="panda_manipulation",
                        executable="gazebo_ready_pose",
                        name="gazebo_ready_pose",
                        output="screen",
                        condition=IfCondition(move_to_ready),
                    )
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="panda_manipulation",
                        executable="gazebo_gripper_smoke_test",
                        name="gazebo_gripper_smoke_test",
                        output="screen",
                        condition=IfCondition(run_gripper_test),
                    )
                ],
            ),
        ]
    )
