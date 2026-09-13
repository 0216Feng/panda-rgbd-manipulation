import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    gui_config = LaunchConfiguration("gui_config")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    world = LaunchConfiguration("world")
    run_smoke_test = LaunchConfiguration("run_smoke_test")
    enable_servo = LaunchConfiguration("enable_servo")
    run_servo_readiness_test = LaunchConfiguration("run_servo_readiness_test")
    obstacle_x = LaunchConfiguration("obstacle_x")
    obstacle_y = LaunchConfiguration("obstacle_y")
    obstacle_z = LaunchConfiguration("obstacle_z")
    obstacle_size_x = LaunchConfiguration("obstacle_size_x")
    obstacle_size_y = LaunchConfiguration("obstacle_size_y")
    obstacle_size_z = LaunchConfiguration("obstacle_size_z")

    package_share = get_package_share_directory("panda_manipulation")
    panda_moveit_share = get_package_share_directory("moveit_resources_panda_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder(
            "panda",
            package_name="moveit_resources_panda_moveit_config",
        )
        .robot_description(
            file_path=os.path.join(package_share, "urdf", "panda_gz.urdf.xacro")
        )
        .trajectory_execution(
            file_path=os.path.join(
                package_share,
                "config",
                "moveit_gazebo_controllers.yaml",
            )
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    gazebo_launch = os.path.join(package_share, "launch", "gazebo_control.launch.py")
    default_world = os.path.join(package_share, "worlds", "panda_table.sdf")
    default_gui_config = os.path.join(
        package_share,
        "config",
        "gazebo_demo.gui.config",
    )
    default_rviz_config = os.path.join(panda_moveit_share, "launch", "moveit.rviz")
    servo_params = {
        "moveit_servo": ParameterBuilder("panda_manipulation")
        .yaml("config/servo_gazebo.yaml")
        .to_dict()
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("gui_config", default_value=default_gui_config),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("run_smoke_test", default_value="true"),
            DeclareLaunchArgument("enable_servo", default_value="false"),
            DeclareLaunchArgument(
                "run_servo_readiness_test",
                default_value="false",
            ),
            DeclareLaunchArgument("obstacle_x", default_value="0.42"),
            DeclareLaunchArgument("obstacle_y", default_value="0.30"),
            DeclareLaunchArgument("obstacle_z", default_value="0.09"),
            DeclareLaunchArgument("obstacle_size_x", default_value="0.12"),
            DeclareLaunchArgument("obstacle_size_y", default_value="0.12"),
            DeclareLaunchArgument("obstacle_size_z", default_value="0.18"),
            DeclareLaunchArgument("static_environment_world", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "gui": gui,
                    "gui_config": gui_config,
                    "world": world,
                    "move_to_ready": "false",
                }.items(),
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
            ),
            Node(
                package="moveit_servo",
                executable="servo_node",
                name="servo_node",
                output="screen",
                parameters=[
                    servo_params,
                    {"update_period": 0.02},
                    {"planning_group_name": "panda_arm"},
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    {"use_sim_time": True},
                ],
                condition=IfCondition(enable_servo),
            ),
            Node(
                package="panda_manipulation",
                executable="scene_manager",
                name="gazebo_moveit_scene_manager",
                output="screen",
                parameters=[
                    {
                        "publish_moveit_scene": True,
                        "include_target_collision": False,
                        "static_environment_world": ParameterValue(
                            LaunchConfiguration("static_environment_world"), value_type=str
                        ),
                        "obstacle_x": obstacle_x,
                        "obstacle_y": obstacle_y,
                        "obstacle_z": obstacle_z,
                        "obstacle_size_x": obstacle_size_x,
                        "obstacle_size_y": obstacle_size_y,
                        "obstacle_size_z": obstacle_size_z,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    {"use_sim_time": True},
                ],
                condition=IfCondition(rviz),
            ),
            TimerAction(
                period=12.0,
                actions=[
                    Node(
                        package="panda_manipulation",
                        executable="moveit_gazebo_smoke_test",
                        name="moveit_gazebo_smoke_test",
                        output="screen",
                        condition=IfCondition(run_smoke_test),
                    )
                ],
            ),
            TimerAction(
                period=12.0,
                actions=[
                    Node(
                        package="panda_manipulation",
                        executable="servo_readiness_test",
                        name="servo_readiness_test",
                        output="screen",
                        parameters=[{"use_sim_time": True}],
                        condition=IfCondition(run_servo_readiness_test),
                    )
                ],
            ),
        ]
    )
