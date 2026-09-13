"""Run the pick-and-place task state machine."""

from __future__ import annotations

import json
from typing import Dict, List, Optional

from .ros_helpers import require_ros2
from .task_state_machine import ManipulationTaskStateMachine, StepResult, TaskState


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from rclpy.node import Node
    from std_msgs.msg import String

    class ManipulationTaskManager(Node):
        def __init__(self) -> None:
            super().__init__("manipulation_task_manager")
            self.declare_parameter("dry_run", True)
            self.declare_parameter("max_retries", 2)
            self.dry_run = bool(self.get_parameter("dry_run").value)
            self.state_machine = ManipulationTaskStateMachine(
                max_retries=int(self.get_parameter("max_retries").value)
            )
            self.latest_candidates: Optional[Dict[str, object]] = None
            self.has_run_current_candidates = False
            self.publisher = self.create_publisher(String, "/task_state", 10)
            self.create_subscription(String, "/grasp_candidates", self.on_grasp_candidates, 10)
            self.timer = self.create_timer(0.25, self.tick)
            self.get_logger().info(f"Task manager started with dry_run={self.dry_run}.")

        def on_grasp_candidates(self, message: String) -> None:
            self.latest_candidates = json.loads(message.data)
            self.has_run_current_candidates = False

        def tick(self) -> None:
            if self.latest_candidates is None or self.has_run_current_candidates:
                return
            self.has_run_current_candidates = True
            report = self.state_machine.run(self._handlers())
            output = String()
            output.data = json.dumps(
                {
                    "final_state": report.final_state.value,
                    "history": report.history,
                    "failures": report.failures,
                    "failure_reason": report.failure_reason,
                }
            )
            self.publisher.publish(output)
            self.get_logger().info(output.data)
            self.state_machine.reset()

        def _handlers(self):
            return {
                TaskState.DETECT_OBJECT: self._detect_object,
                TaskState.PLAN_TO_PRE_GRASP: lambda: self._plan("pre_grasp_pose"),
                TaskState.APPROACH: lambda: self._execute("approach"),
                TaskState.CLOSE_GRIPPER: lambda: self._execute("close_gripper"),
                TaskState.LIFT: lambda: self._execute("lift"),
                TaskState.PLAN_TO_PLACE: lambda: self._plan("place_pose"),
                TaskState.OPEN_GRIPPER: lambda: self._execute("open_gripper"),
                TaskState.RETURN_HOME: lambda: self._execute("return_home"),
            }

        def _detect_object(self) -> StepResult:
            if not self.latest_candidates:
                return StepResult(False, "no grasp candidates available")
            if self.latest_candidates.get("selected_candidate_id") is None:
                return StepResult(False, "no reachable grasp candidate")
            return StepResult(True)

        def _plan(self, target_key: str) -> StepResult:
            if self.dry_run:
                return StepResult(True)
            return StepResult(False, f"MoveIt2 planning adapter for {target_key} is not connected")

        def _execute(self, action_name: str) -> StepResult:
            if self.dry_run:
                return StepResult(True)
            return StepResult(False, f"execution adapter for {action_name} is not connected")

    rclpy.init(args=args)
    node = ManipulationTaskManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
