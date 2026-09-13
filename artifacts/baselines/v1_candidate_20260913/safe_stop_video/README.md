# Gazebo Wrench Safe-Stop Video Evidence

This compact bundle records one current-source Gazebo safety trial captured on
2026-09-13. A bounded `[0, 16, 0] N` wrench was applied for `100 ms` during
Servo place-descent stage 3. The wrench lifecycle reached `APPLIED` and
`CLEARED`; the payload/contact watchdog stopped the matching stage and
published a zero command in `107.8 ms`, below the `200 ms` gate.

The manipulation task intentionally ended in `FAILED`. The independent safety
verdict is `SAFE_STOP_PASS`; generic task failure is not accepted as a safety
pass.

- `payload_transfer_comparison.csv`: machine-readable trial row.
- `physical_disturbance_safety.json`: aggregate safety verdict and latency.
- `physical_disturbance_safety.md`: human-readable report.
- `experiment_config.json`: exact runner arguments.
- `source_manifest.json`: hashes for the evaluated source snapshot.
- `video_metadata.json`: codec, duration, public path, and content hash.

The corresponding H.264 clip and poster are stored in
`docs/assets/demo/gazebo_safe_stop.mp4` and
`docs/assets/demo/gazebo_safe_stop_poster.png`.
