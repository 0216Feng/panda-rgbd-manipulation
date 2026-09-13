"""Record pick plan pipeline results as CSV and Markdown metrics."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean
from typing import Dict, List

from .ros_helpers import require_ros2


def is_terminal_result(result: Dict[str, object]) -> bool:
    return result.get("final_state") in {"SUCCESS", "FAILED"}


def summarize_results(results: List[Dict[str, object]]) -> Dict[str, object]:
    results = [result for result in results if is_terminal_result(result)]
    total = len(results)
    successes = [result for result in results if result.get("final_state") == "SUCCESS"]
    failures = [result for result in results if result.get("final_state") != "SUCCESS"]
    execution_trials = [result for result in results if result.get("execute_trajectories")]
    execution_successes = [
        result for result in execution_trials if result.get("final_state") == "SUCCESS"
    ]
    backend_counts: Dict[str, int] = {}
    for result in execution_trials:
        backend = str(result.get("ompl_execution_backend", "unknown"))
        backend_counts[backend] = backend_counts.get(backend, 0) + 1
    failed_steps: Dict[str, int] = {}
    failure_reasons = []
    for result in failures:
        failed_step = final_failed_step(result)
        failed_steps[failed_step] = failed_steps.get(failed_step, 0) + 1
        failure_reasons.append(str(result.get("failure_reason", "")))

    step_counts = [len(result.get("steps", [])) for result in results]
    return {
        "total": total,
        "successes": len(successes),
        "failures": len(failures),
        "success_rate": len(successes) / total if total else 0.0,
        "avg_completed_steps": mean(step_counts) if step_counts else 0.0,
        "execution_trials": len(execution_trials),
        "execution_successes": len(execution_successes),
        "execution_success_rate": (
            len(execution_successes) / len(execution_trials) if execution_trials else 0.0
        ),
        "execution_backends": backend_counts,
        "failed_steps": failed_steps,
        "failure_reasons": failure_reasons,
    }


def final_failed_step(result: Dict[str, object]) -> str:
    failure_reason = result.get("failure_reason")
    if isinstance(failure_reason, str) and ":" in failure_reason:
        return failure_reason.split(":", 1)[0]

    steps = result.get("steps", [])
    for step in reversed(steps):
        if not step.get("success", False):
            return str(step.get("step", "unknown"))
    return "unknown"


def render_markdown(summary: Dict[str, object]) -> str:
    lines = [
        "# Pick Plan Benchmark Report",
        "",
        f"- Trials: {summary['total']}",
        f"- Successes: {summary['successes']}",
        f"- Failures: {summary['failures']}",
        f"- Success rate: {summary['success_rate']:.1%}",
        f"- Average completed steps: {summary['avg_completed_steps']:.2f}",
        f"- Execution trials: {summary.get('execution_trials', 0)}",
    ]
    if summary.get("execution_trials", 0):
        lines.extend(
            [
                f"- Execution successes: {summary.get('execution_successes', 0)}",
                f"- Execution success rate: {summary.get('execution_success_rate', 0.0):.1%}",
                f"- Execution backends: {summary.get('execution_backends', {})}",
            ]
        )
    lines.extend(
        [
            "",
            "## Failed Steps",
            "",
        ]
    )
    failed_steps = summary["failed_steps"]
    if failed_steps:
        for step, count in sorted(failed_steps.items()):
            lines.append(f"- {step}: {count}")
    else:
        lines.append("- None")
    lines.extend(["", "## Failure Reasons", ""])
    failure_reasons = summary.get("failure_reasons", [])
    if failure_reasons:
        for index, reason in enumerate(failure_reasons, start=1):
            lines.append(f"{index}. {reason}")
    else:
        lines.append("- None")
    lines.extend(
        [
            "",
            "## Resume Metric",
            "",
            resume_metric_sentence(summary),
        ]
    )
    return "\n".join(lines) + "\n"


def resume_metric_sentence(summary: Dict[str, object]) -> str:
    if summary.get("execution_trials", 0):
        return (
            "在中心工作区 {trials} 组目标位姿 execution benchmark 中，基于 MoveIt2 "
            "实现 MoveGroup + Cartesian 执行闭环，达到 {rate:.1%} 执行成功率，"
            "并输出失败阶段分布与 Markdown/CSV 实验报告。"
        ).format(trials=summary["total"], rate=summary.get("execution_success_rate", 0.0))
    return (
        "在中心工作区 {trials} 组目标位姿 benchmark 中，基于 MoveIt2 实现 "
        "OMPL + Cartesian 混合抓取规划，达到 {rate:.1%} plan-only 成功率，"
        "并输出失败阶段分布与 Markdown/CSV 实验报告。"
    ).format(trials=summary["total"], rate=summary["success_rate"])


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    class PickPlanBenchmarkRecorder(Node):
        def __init__(self) -> None:
            super().__init__("pick_plan_benchmark_recorder")
            self.declare_parameter("expected_trials", 9)
            self.declare_parameter("output_csv", "pick_plan_benchmark.csv")
            self.declare_parameter("output_markdown", "pick_plan_benchmark.md")

            self.expected_trials = int(self.get_parameter("expected_trials").value)
            self.output_csv = str(self.get_parameter("output_csv").value)
            self.output_markdown = str(self.get_parameter("output_markdown").value)
            self.results: List[Dict[str, object]] = []

            qos = QoSProfile(depth=10)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.summary_publisher = self.create_publisher(String, "/pick_plan_benchmark_summary", qos)
            self.create_subscription(String, "/pick_plan_result", self.on_result, 10)
            self.get_logger().info(f"Recording {self.expected_trials} pick plan benchmark trials.")

        def on_result(self, message: String) -> None:
            result = json.loads(message.data)
            if not is_terminal_result(result):
                self.get_logger().info(f"Ignoring non-terminal result: {result.get('final_state')}")
                return
            self.results.append(result)
            self.write_outputs()
            self.get_logger().info(
                f"Recorded trial {len(self.results)}/{self.expected_trials}: {result.get('final_state')}"
            )
            if len(self.results) >= self.expected_trials:
                self.publish_summary()

        def write_outputs(self) -> None:
            rows = []
            for index, result in enumerate(self.results, start=1):
                steps = result.get("steps", [])
                failed_step = ""
                if result.get("final_state") != "SUCCESS":
                    failed_step = final_failed_step(result)
                rows.append(
                    {
                        "trial": index,
                        "final_state": result.get("final_state", ""),
                        "completed_steps": len(steps),
                        "failed_step": failed_step,
                        "failure_reason": result.get("failure_reason", ""),
                    }
                )

            with Path(self.output_csv).open("w", newline="", encoding="utf-8") as csv_file:
                writer = csv.DictWriter(
                    csv_file,
                    fieldnames=["trial", "final_state", "completed_steps", "failed_step", "failure_reason"],
                )
                writer.writeheader()
                writer.writerows(rows)

            summary = summarize_results(self.results)
            Path(self.output_markdown).write_text(render_markdown(summary), encoding="utf-8")

        def publish_summary(self) -> None:
            summary = summarize_results(self.results)
            message = String()
            message.data = json.dumps(summary)
            self.summary_publisher.publish(message)
            self.get_logger().info(message.data)

    rclpy.init(args=args)
    node = PickPlanBenchmarkRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
