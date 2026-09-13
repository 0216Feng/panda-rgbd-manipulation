"""Run the benchmark model without ROS2 and write local report artifacts."""

from __future__ import annotations

import argparse
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PACKAGE_ROOT = os.path.join(REPO_ROOT, "src", "panda_manipulation")
sys.path.insert(0, PACKAGE_ROOT)

from panda_manipulation.benchmark_metrics import aggregate_by_planner, render_markdown
from panda_manipulation.planner_benchmark_runner import synthetic_runs, write_reports


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials-per-planner", type=int, default=30)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--csv", default="benchmark_results.csv")
    parser.add_argument("--markdown", default="benchmark_report.md")
    args = parser.parse_args()

    runs = synthetic_runs(trials_per_planner=args.trials_per_planner, seed=args.seed)
    write_reports(runs, args.csv, args.markdown)
    print(render_markdown(aggregate_by_planner(runs)))


if __name__ == "__main__":
    main()
