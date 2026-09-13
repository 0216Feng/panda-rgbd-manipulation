# Experiment Artifacts

Generated benchmark data is kept outside the project root so source files and
entry points remain easy to scan.

## Layout

- `baselines/`: curated benchmark reports referenced by the main README.
- `current/`: recent diagnostic runs that support active development.
- `runs/`: destination for new local experiments; ignored by Git.
- `archive/`: historical experiments grouped by topic; ignored by Git.
- `archive_manifest_20260829.csv`: original top-level name, new path, type,
  byte size, and modification time for every item moved during cleanup.

No historical experiment was deleted during the 2026-08-29 cleanup.

## New Runs

Give each experiment its own directory instead of writing reports into the
project root:

```bash
mkdir -p artifacts/runs/my_experiment
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 \
  --csv artifacts/runs/my_experiment/results.csv \
  --markdown artifacts/runs/my_experiment/report.md \
  --svg artifacts/runs/my_experiment/chart.svg \
  --log-dir artifacts/runs/my_experiment/logs
```

Promote a result from `runs/` to `baselines/` only after its sample size,
configuration, success criteria, and failure accounting are documented.
