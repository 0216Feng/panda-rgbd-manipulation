[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](README.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](README.zh-CN.md)

# Demo Capture Provenance

These screenshots were captured from the live RViz window during one
fresh-world Gazebo Harmonic run on 2026-09-11. The run used markerless RGB-D
localization, the `center_tall_barrier` challenge, RRTConnect, physical finger
control, contact-based payload monitoring, and independent Gazebo outcome
validation.

`obstacle_pick_sequence.gif` is an eight-frame summary from the same run. The
three PNG files preserve higher-resolution stills for inspection and reuse in
the README, technical presentations, or design reviews.

`gazebo_obstacle_pick.mp4` is a separate fresh-world run captured on 2026-09-12
from the inactive-by-default opposite-side sensor in the Gazebo world. This
view keeps the target and finger contact visible instead of placing the tall
barriers between the camera and grasp point. The published H.264 clip is 38.2
seconds at 960x540 and 15 fps. It is a 2x playback of the complete 76.2-second
sensor stream; no task phase or failed attempt was removed.
`gazebo_obstacle_pick_poster.png` is a frame from the lift phase.
`gazebo_obstacle_pick_preview.webp` is the complete clip resampled to 720x405
at 7.5 fps for inline GitHub README playback. Its 287 frames preserve the full
38.2-second timeline while keeping the asset below 700 KiB. The MP4 remains
the higher-frame-rate source.

This is physical-contact simulation evidence, not real-robot footage. Real
hardware demonstration remains a separate roadmap item requiring a robot,
calibration, driver integration, workspace safety controls, and an emergency
stop procedure.

`gazebo_safe_stop.mp4` is a third fresh-world run captured on 2026-09-13. It
shows the robot halt after an intentional real Gazebo wrench disturbance during
Servo descent. The 13.8-second H.264 clip is 2x playback at 960x540 and 15 fps;
`gazebo_safe_stop_poster.png` shows the stopped loaded state, and
`gazebo_safe_stop_preview.webp` is the complete 720x405, 7.5 fps inline README
preview.

## Result

- Pipeline and validator: `SUCCESS`
- Lift delta: `0.1258 m`
- Final placement error: `0.0041 m`
- Final object tilt: `0.00 deg`
- Direct path blocked by the PlanningScene: yes
- RGB-D mean target error: `0.0011 m` across 10 samples
- OMPL planning time: `0.0441 s`
- OMPL joint path length: `6.3205 rad`

The compact result, configuration, and source fingerprint are retained in
`artifacts/baselines/v1_candidate_20260911/portfolio_capture/`.

The video run independently passed with `0.4567 m` maximum lift, `0.0115 m`
placement error, `0.00 deg` final tilt, RGB-D mean target error `0.0011 m`, and
a collision-aware direct path blocked in `1/1` trial. Its CSV, report,
configuration, source fingerprint, and video metadata are retained in
`artifacts/baselines/v1_candidate_20260912/gazebo_video/`.

The safe-stop run applied `[0, 16, 0] N` for `100 ms` during descent stage 3.
The wrench was applied and cleared, the payload/contact watchdog stopped the
matching stage, and the zero-command latency was `107.8 ms` against a `200 ms`
limit. The task terminal state is intentionally `FAILED`; the independent
safety verdict is `SAFE_STOP_PASS`. Its source-bound evidence is retained in
`artifacts/baselines/v1_candidate_20260913/safe_stop_video/`.

## Reproduction

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 \
  --challenge-obstacles \
  --challenge-scenario center_tall_barrier \
  --rgbd-perception \
  --planner-id RRTConnectkConfigDefault \
  --grasp-height-offset 0.115 \
  --grasp-pitch-offset-deg -20 \
  --loaded-path-precheck \
  --transfer-diagnostics \
  --rviz --gazebo-gui \
  --timeout-s 360 \
  --hold-after-result-s 90 \
  --csv artifacts/runs/portfolio_capture/results.csv \
  --markdown artifacts/runs/portfolio_capture/report.md \
  --svg artifacts/runs/portfolio_capture/report.svg \
  --log-dir artifacts/runs/portfolio_capture/logs
```

The screenshots are documentary assets. Machine-readable CSV/JSON evidence is
the source of truth for quantitative claims.

## Gazebo Video Reproduction

The recorder and benchmark share the benchmark's isolated ROS domain. The
opposite-side showcase camera and its dedicated `ros_gz_bridge` process are activated only by
this command, so ordinary benchmarks do not pay the extra rendering cost.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/run_gazebo_camera_video.sh \
  artifacts/runs/gazebo_camera_video
```

The command writes the raw MP4, CSV, Markdown/SVG report, source/configuration
sidecars, and logs into the selected run directory. The committed H.264 file
was produced from the raw 640x360, 15 fps recording with:

```bash
ffmpeg -i gazebo_rgb_camera_raw.mp4 \
  -vf "setpts=0.5*PTS,scale=960:540:flags=lanczos,eq=contrast=1.03:saturation=1.08" \
  -an -c:v libx264 -crf 22 -pix_fmt yuv420p -movflags +faststart \
  gazebo_obstacle_pick.mp4
```

## Safe-Stop Video Reproduction

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/run_safe_stop_video.sh \
  artifacts/runs/safe_stop_video
```

The script can use a native Linux FFmpeg or discover an existing Windows
FFmpeg from WSL. It rejects a recording unless the real wrench lifecycle,
watchdog stage, zero-command latency, and nonempty video all pass.
