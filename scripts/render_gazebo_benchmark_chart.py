"""Render an SVG planner comparison from an existing Gazebo benchmark CSV."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "panda_manipulation"
sys.path.insert(0, str(PACKAGE_ROOT))

from panda_manipulation.physics_benchmark import (  # noqa: E402
    read_report_rows,
    summarize,
    write_svg_report,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Render a dependency-free SVG from a Gazebo benchmark CSV."
    )
    parser.add_argument("csv", help="input benchmark CSV")
    parser.add_argument("svg", help="output SVG")
    args = parser.parse_args()

    rows = read_report_rows(args.csv)
    summary = summarize(rows)
    write_svg_report(summary, args.svg)
    print(f"Rendered {args.svg} from {summary['total']} trial(s).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
