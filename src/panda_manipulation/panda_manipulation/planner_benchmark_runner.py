"""Run planner benchmark experiments and publish/report aggregate metrics."""

from __future__ import annotations

import json
import random
from pathlib import Path
from typing import Iterable, List

from .benchmark_metrics import PlannerRun, aggregate_by_planner, render_markdown, write_csv
from .ros_helpers import require_ros2


PLANNER_PROFILES = {
    "RRTConnect": {"base_time": 0.45, "success": 0.92, "length": 0.82},
    "PRM": {"base_time": 0.70, "success": 0.86, "length": 0.78},
    "RRTstar": {"base_time": 1.15, "success": 0.80, "length": 0.72},
}

SCENARIOS = ["center_clear", "left_obstacle", "right_obstacle"]


def synthetic_runs(trials_per_planner: int = 30, seed: int = 7) -> List[PlannerRun]:
    rng = random.Random(seed)
    runs: List[PlannerRun] = []
    for planner_id, profile in PLANNER_PROFILES.items():
        for trial in range(trials_per_planner):
            scenario_id = SCENARIOS[trial % len(SCENARIOS)]
            clutter_penalty = 0.0 if scenario_id == "center_clear" else 0.12
            success = rng.random() < float(profile["success"]) - clutter_penalty
            planning_time = float(profile["base_time"]) + rng.uniform(0.02, 0.35) + clutter_penalty
            length = float(profile["length"]) + rng.uniform(-0.08, 0.14) + clutter_penalty
            execution_time = 8.0 + length * 6.0 + rng.uniform(0.0, 2.0)
            runs.append(
                PlannerRun(
                    planner_id=planner_id,
                    scenario_id=scenario_id,
                    success=success,
                    planning_time_s=planning_time,
                    trajectory_length_m=max(0.1, length),
                    execution_time_s=execution_time,
                    failure_reason="" if success else rng.choice(["ik_failed", "collision", "timeout"]),
                )
            )
    return runs


def write_reports(
    runs: Iterable[PlannerRun],
    output_csv: str,
    output_markdown: str,
) -> str:
    runs = list(runs)
    write_csv(output_csv, runs)
    markdown = render_markdown(aggregate_by_planner(runs))
    Path(output_markdown).write_text(markdown, encoding="utf-8")
    return markdown


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from rclpy.node import Node
    from std_msgs.msg import String

    class PlannerBenchmarkRunner(Node):
        def __init__(self) -> None:
            super().__init__("planner_benchmark_runner")
            self.declare_parameter("trials_per_planner", 30)
            self.declare_parameter("seed", 7)
            self.declare_parameter("output_csv", "benchmark_results.csv")
            self.declare_parameter("output_markdown", "benchmark_report.md")
            self.publisher = self.create_publisher(String, "/benchmark_result", 10)
            self.timer = self.create_timer(0.5, self.run_once)
            self.completed = False

        def run_once(self) -> None:
            if self.completed:
                return
            self.completed = True
            runs = synthetic_runs(
                trials_per_planner=int(self.get_parameter("trials_per_planner").value),
                seed=int(self.get_parameter("seed").value),
            )
            markdown = write_reports(
                runs,
                str(self.get_parameter("output_csv").value),
                str(self.get_parameter("output_markdown").value),
            )
            summary = aggregate_by_planner(runs)
            message = String()
            message.data = json.dumps(summary)
            self.publisher.publish(message)
            self.get_logger().info("Planner benchmark completed.")
            self.get_logger().info("\n" + markdown)

    rclpy.init(args=args)
    node = PlannerBenchmarkRunner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
