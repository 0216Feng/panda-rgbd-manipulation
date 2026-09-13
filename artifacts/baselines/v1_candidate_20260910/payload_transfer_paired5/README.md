# Current-Source Paired Payload Evidence

This compact public bundle contains five unloaded/loaded pairs, ten task runs in
total. The runs used the same fixed obstacle task, Cartesian descent profile,
controller settings, and guarded source/configuration fingerprint.

| Mode | Success | Mean max transfer error | P95 max transfer error | Mean RMS transfer error | Mean payload drift |
| --- | ---: | ---: | ---: | ---: | ---: |
| Unloaded | 5/5 | 0.00019 rad | 0.00025 rad | 0.000010 rad | N/A |
| Loaded | 5/5 | 0.00211 rad | 0.00801 rad | 0.000034 rad | 0.0056 m |

All five pre-descent pairs matched under the 0.05 rad gate. Mean joint delta was
0.00473 rad and maximum delta was 0.00499 rad. Each mode's 100% point estimate
has a 56.6%-100% Wilson 95% interval because the sample contains only five runs.

## Files

- `results.csv`: every trial row; local telemetry, rosbag, and log path fields
  are intentionally blank in the public copy.
- `report.md`: generated aggregate report.
- `summary.json`: machine-readable aggregate and pair-state diagnostics.
- `experiment_config.json`: complete runner arguments and aggregate source hash.
- `source_manifest.json`: per-file SHA256 values for the guarded source set.

The full local run directory includes telemetry, logs, and rosbags. Those files
are excluded from Git because they are large and contain machine-specific paths.
The public CSV retains all outcomes and metrics needed to recompute this report.
