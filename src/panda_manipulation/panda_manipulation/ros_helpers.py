"""ROS import helpers with friendly errors outside ROS2 environments."""


def require_ros2():
    try:
        import rclpy  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 Python modules are not available. Run this node inside a sourced ROS2 Jazzy environment."
        ) from exc
    return rclpy
