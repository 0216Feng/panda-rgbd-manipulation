$ErrorActionPreference = "Stop"

python -m unittest discover src/panda_manipulation/test
python scripts/run_synthetic_benchmark.py --trials-per-planner 3 --csv benchmark_results.csv --markdown benchmark_report.md
