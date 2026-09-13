# Gazebo Physical Disturbance Safety

A pass requires the real Gazebo wrench to be applied and cleared, the Servo watchdog to stop the matching descent stage for measured contact loss or payload drift, and the zero command to meet the configured latency limit. Task failure alone is not a pass.

- Safe stops: 1/1 (100.0% (20.7%-100.0%)).
- Disturbance lifecycle: 1 applied, 1 cleared.
- Configured per-trial stop-latency limits: 0.200 s.
- All observed stops meet the strictest 0.200 s limit: yes.

## Disturbance Configurations

| Force [x, y, z] | Duration | Stage delay | Trials | Safe stops |
| --- | ---: | ---: | ---: | ---: |
| [0.0, 16.0, 0.0] N | 0.100 s | 0.500 s | 1 | 1 |

## Stage Matrix

| Stage | Trials | Applied | Cleared | Safe stops | Mean / max latency |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 3 | 1 | 1 | 1 | 1 | 0.1712 / 0.1712 s |
