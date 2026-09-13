from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    trajectory_duration_s = LaunchConfiguration("trajectory_duration_s")
    gazebo_control_launch = PathJoinSubstitution(
        [FindPackageShare("panda_manipulation"), "launch", "gazebo_control.launch.py"]
    )
    tracking_test = Node(
        package="panda_manipulation",
        executable="gazebo_arm_tracking_test",
        name="gazebo_arm_tracking_test",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "trajectory_duration_s": trajectory_duration_s,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("trajectory_duration_s", default_value="10.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_control_launch),
                launch_arguments={
                    "gui": gui,
                    "move_to_ready": "false",
                    "run_gripper_test": "false",
                }.items(),
            ),
            TimerAction(period=9.0, actions=[tracking_test]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=tracking_test,
                    on_exit=[Shutdown(reason="arm tracking diagnostic completed")],
                )
            ),
        ]
    )
