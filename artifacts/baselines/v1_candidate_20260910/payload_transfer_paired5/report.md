# Payload Transfer Diagnostics

The unloaded and loaded runs use the same fixed obstacle task and controller settings.

| Mode | Trials | Success rate (95% CI) | Mean / median max error | P95 max error | Mean / median RMS error | Mean payload drift | Mean time |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| unloaded | 5 | 100.0% (56.6%-100.0%) | 0.00019 / 0.00020 rad | 0.00025 rad | 0.00001 / 0.00001 rad | N/A | 125.8 s |
| loaded | 5 | 100.0% (56.6%-100.0%) | 0.00211 / 0.00013 rad | 0.00801 rad | 0.00003 / 0.00001 rad | 0.0056 m | 149.6 s |

## Interpretation Gate

- Loaded/unloaded RMS tracking-error ratio: 3.53x
- Matched pre-descent states: 5/5 pairs (required max joint delta <= 0.050 rad).
- Pair start-state joint delta: mean 0.00473 rad, max 0.00499 rad.
- Matched-pair RMS tracking-error ratio: 3.53x
- Loaded-minus-unloaded mean RMS error: 0.000025 rad; matched pairs: 0.000025 rad.
- Ratio caution: unloaded RMS is below 0.0001 rad, so the relative multiplier is numerically sensitive; prioritize absolute loaded error, P95, task success, and payload drift.
- Attribute differences to payload only from matched-state pairs.
- Unloaded tracking failures point to controller timing or execution.
- Loaded-only degradation points to grasp force, friction, depth, or wrist pose.
- Normal tracking with rising hand-object drift points to contact or collision modeling.
