"""Invalidate an executing trajectory when a sensed obstacle enters its path."""

from __future__ import annotations

import json
from math import sqrt
from typing import Dict, List, Optional, Sequence

from .ros_helpers import require_ros2


def nearest_trajectory_index(
    trajectory_rows: Sequence[Sequence[float]],
    current_positions: Sequence[float],
) -> int:
    if not trajectory_rows:
        return 0
    if not current_positions:
        return 0
    return min(
        range(len(trajectory_rows)),
        key=lambda index: sqrt(
            sum(
                (float(value) - float(current)) ** 2
                for value, current in zip(trajectory_rows[index], current_positions)
            )
        ),
    )


def sample_remaining_indices(start: int, total: int, maximum_samples: int) -> List[int]:
    if total <= 0:
        return []
    start = min(max(0, start), total - 1)
    count = total - start
    maximum_samples = max(1, maximum_samples)
    if count <= maximum_samples:
        return list(range(start, total))
    indices = {
        round(start + offset * (count - 1) / (maximum_samples - 1))
        for offset in range(maximum_samples)
    }
    return sorted(indices)


def trajectory_collision_lead_s(
    trajectory_times_s: Sequence[float],
    start_index: int,
    collision_index: int,
) -> float:
    """Return trajectory time remaining until a sampled collision state."""
    if not trajectory_times_s:
        return 0.0
    last = len(trajectory_times_s) - 1
    start = min(max(0, start_index), last)
    collision = min(max(start, collision_index), last)
    return max(
        0.0,
        float(trajectory_times_s[collision]) - float(trajectory_times_s[start]),
    )


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from moveit_msgs.msg import DisplayTrajectory, RobotState
    from moveit_msgs.srv import GetStateValidity
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    class TrajectorySafetyMonitor(Node):
        def __init__(self) -> None:
            super().__init__("trajectory_safety_monitor")
            self.declare_parameter("state_validity_service", "/check_state_validity")
            self.declare_parameter("planning_group", "panda_arm")
            self.declare_parameter("scene_settle_s", 0.20)
            self.declare_parameter("maximum_state_samples", 30)
            self.declare_parameter("minimum_check_period_s", 0.20)
            self.declare_parameter("monitored_step_prefixes", ["ompl_"])
            self.group_name = str(self.get_parameter("planning_group").value)
            self.scene_settle_ns = int(
                max(0.0, float(self.get_parameter("scene_settle_s").value)) * 1e9
            )
            self.maximum_samples = max(
                2,
                int(self.get_parameter("maximum_state_samples").value),
            )
            self.minimum_check_period_ns = int(
                max(0.05, float(self.get_parameter("minimum_check_period_s").value)) * 1e9
            )
            self.monitored_step_prefixes = tuple(
                str(value)
                for value in self.get_parameter("monitored_step_prefixes").value
            )
            self.client = self.create_client(
                GetStateValidity,
                str(self.get_parameter("state_validity_service").value),
            )
            self.publisher = self.create_publisher(String, "/dynamic_replan_request", 10)
            self.event_publisher = self.create_publisher(String, "/trajectory_safety_event", 10)
            self.create_subscription(
                DisplayTrajectory,
                "/display_planned_path",
                self.on_display_trajectory,
                10,
            )
            self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
            self.create_subscription(String, "/pick_demo_state", self.on_demo_state, 10)
            self.create_subscription(
                String,
                "/dynamic_obstacle_detection",
                self.on_obstacle_detection,
                10,
            )
            self.latest_trajectory = None
            self.latest_joint_state: Optional[JointState] = None
            self.active_step: Optional[str] = None
            self.obstacle_payload: Optional[Dict[str, object]] = None
            self.pending_check_ns: Optional[int] = None
            self.last_check_ns = 0
            self.check_busy = False
            self.hazard_sent = False
            self.check_indices: List[int] = []
            self.check_step: Optional[str] = None
            self.check_started_ns = 0
            self.check_start_index = 0
            self.check_times_s: List[float] = []
            self.timer = self.create_timer(0.05, self.maybe_start_check)

        def on_display_trajectory(self, message: DisplayTrajectory) -> None:
            if message.trajectory:
                self.latest_trajectory = message.trajectory[-1]

        def on_joint_state(self, message: JointState) -> None:
            self.latest_joint_state = message

        def on_demo_state(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            step = payload.get("step")
            mode = payload.get("mode")
            monitored = isinstance(step, str) and any(
                step.startswith(prefix) for prefix in self.monitored_step_prefixes
            )
            if mode == "executing" and monitored:
                self.active_step = step
                self.latest_trajectory = None
                self.hazard_sent = False
                self.pending_check_ns = self.get_clock().now().nanoseconds + self.scene_settle_ns
            elif mode == "executing":
                self.active_step = None
                self.pending_check_ns = None
            elif mode == "executed" and step == self.active_step:
                self.active_step = None
                self.pending_check_ns = None

        def on_obstacle_detection(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if payload.get("state") != "TRACKING":
                return
            self.obstacle_payload = payload
            if (
                self.active_step is not None
                and not self.hazard_sent
                and not self.check_busy
                and self.pending_check_ns is None
            ):
                self.pending_check_ns = self.get_clock().now().nanoseconds + self.scene_settle_ns

        def maybe_start_check(self) -> None:
            now_ns = self.get_clock().now().nanoseconds
            if (
                self.check_busy
                or self.hazard_sent
                or self.active_step is None
                or self.pending_check_ns is None
                or now_ns < self.pending_check_ns
                or now_ns - self.last_check_ns < self.minimum_check_period_ns
                or self.latest_trajectory is None
                or self.latest_joint_state is None
                or not self.client.service_is_ready()
            ):
                return
            trajectory = self.latest_trajectory.joint_trajectory
            if not trajectory.points:
                return
            current_map = dict(
                zip(self.latest_joint_state.name, self.latest_joint_state.position)
            )
            if any(name not in current_map for name in trajectory.joint_names):
                return
            rows = [list(point.positions) for point in trajectory.points]
            current = [current_map[name] for name in trajectory.joint_names]
            start = nearest_trajectory_index(rows, current)
            self.check_start_index = start
            self.check_times_s = [
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) / 1e9
                for point in trajectory.points
            ]
            self.check_indices = sample_remaining_indices(
                start,
                len(rows),
                self.maximum_samples,
            )
            self.check_busy = True
            self.check_step = self.active_step
            self.check_started_ns = now_ns
            self.last_check_ns = now_ns
            self.pending_check_ns = None
            self.check_next_state()

        def check_next_state(self) -> None:
            if not self.check_indices or self.active_step != self.check_step:
                self.check_busy = False
                return
            index = self.check_indices.pop(0)
            trajectory = self.latest_trajectory.joint_trajectory
            point = trajectory.points[index]
            current = self.latest_joint_state
            position_map = dict(zip(current.name, current.position))
            for name, value in zip(trajectory.joint_names, point.positions):
                position_map[name] = value
            state = RobotState()
            state.joint_state.header.stamp = self.get_clock().now().to_msg()
            state.joint_state.name = list(position_map)
            state.joint_state.position = [position_map[name] for name in state.joint_state.name]
            state.is_diff = False
            request = GetStateValidity.Request()
            request.robot_state = state
            request.group_name = self.group_name
            future = self.client.call_async(request)
            future.add_done_callback(
                lambda done_future: self.on_state_validity(done_future, index)
            )

        def on_state_validity(self, future, index: int) -> None:
            if self.active_step != self.check_step:
                self.check_busy = False
                return
            try:
                response = future.result()
            except Exception as exc:
                self.check_busy = False
                self.get_logger().warn(f"State-validity request failed: {exc}")
                return
            if not response.valid:
                now_ns = self.get_clock().now().nanoseconds
                payload = {
                    "event": "trajectory_invalidated",
                    "step": self.active_step,
                    "trajectory_index": index,
                    "collision_lead_time_s": trajectory_collision_lead_s(
                        self.check_times_s,
                        self.check_start_index,
                        index,
                    ),
                    "check_latency_s": (now_ns - self.check_started_ns) / 1e9,
                    "obstacle": self.obstacle_payload,
                }
                message = String()
                message.data = json.dumps(payload)
                self.publisher.publish(message)
                self.event_publisher.publish(message)
                self.get_logger().warn(message.data)
                self.hazard_sent = True
                self.check_busy = False
                return
            self.check_next_state()

    rclpy.init(args=args)
    node = TrajectorySafetyMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
