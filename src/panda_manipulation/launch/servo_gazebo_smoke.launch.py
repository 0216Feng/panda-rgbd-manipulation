from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    moveit_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panda_manipulation"),
                    "launch",
                    "moveit_gazebo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gui": "false",
            "rviz": "false",
            "run_smoke_test": "false",
            "enable_servo": "true",
            "run_servo_readiness_test": "false",
        }.items(),
    )
    readiness_test = Node(
        package="panda_manipulation",
        executable="servo_readiness_test",
        name="servo_readiness_test",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription(
        [
            moveit_gazebo,
            TimerAction(period=12.0, actions=[readiness_test]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=readiness_test,
                    on_exit=[
                        TimerAction(
                            period=1.0,
                            actions=[Shutdown(reason="Servo readiness test completed")],
                        )
                    ],
                )
            ),
        ]
    )
