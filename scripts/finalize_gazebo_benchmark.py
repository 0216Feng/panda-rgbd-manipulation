"""Rebuild benchmark reports with failure evidence from saved launch logs."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "panda_manipulation"
sys.path.insert(0, str(PACKAGE_ROOT))

from panda_manipulation.physics_benchmark import (  # noqa: E402
    read_report_rows,
    reclassify_infrastructure_failures,
    write_reports,
    write_svg_report,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Finalize a Gazebo benchmark using its saved trial logs."
    )
    parser.add_argument("csv", help="benchmark CSV to update")
    parser.add_argument("markdown", help="Markdown report to rebuild")
    parser.add_argument("log_dir", help="directory containing trial_NNN.log files")
    parser.add_argument(
        "--svg",
        default=None,
        help="SVG report path (defaults to the Markdown path with .svg)",
    )
    args = parser.parse_args()

    rows = read_report_rows(args.csv)
    classified_rows = reclassify_infrastructure_failures(rows, args.log_dir)
    summary = write_reports(classified_rows, args.csv, args.markdown)
    svg_path = args.svg or str(Path(args.markdown).with_suffix(".svg"))
    write_svg_report(summary, svg_path)

    print(
        f"Finalized {summary['total']} trials: raw "
        f"{summary['successes']}/{summary['total']} "
        f"({summary['success_rate']:.1%}), valid-start "
        f"{summary['valid_start_successes']}/{summary['valid_start_trials']} "
        f"({summary['valid_start_success_rate']:.1%})."
    )
    print(f"Reports: {args.csv}, {args.markdown}, {svg_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
