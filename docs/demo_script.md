[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](demo_script.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](demo_script.zh-CN.md)

# Demo Script

## Main 90-Second Demo

Build once, then start the markerless RGB-D physical pick:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/run_rgbd_markerless_pick.sh
```

Record Gazebo and RViz side by side:

1. RViz shows the RGB-D point cloud, accepted target points, fused object pose,
   PlanningScene, and planned trajectory.
2. Gazebo shows the red target cube, table, obstacles, Panda, and both fingers.
3. The arm moves to pre-grasp, performs the final approach, closes both fingers,
   and physically lifts the cube.
4. The transfer avoids the PlanningScene geometry, descends slowly, releases the
   cube upright, retreats, and returns home.
5. End on the structured pipeline and physical-validator SUCCESS records.

Useful terminal views:

```bash
ros2 topic echo /pick_plan_result --once --full-length
ros2 topic echo /gazebo_pick_validation --once --full-length
ros2 topic echo /rgbd_target_detection --once --full-length
```

For a clean third-person Gazebo-only recording with synchronized CSV/report
evidence, use:

```bash
bash scripts/run_gazebo_camera_video.sh \
  artifacts/runs/gazebo_camera_video
```

This activates the otherwise dormant `/showcase_camera/image_raw` sensor,
bridges it into the benchmark's isolated ROS domain, and records the full trial.
The perception pipeline continues to consume the separate overhead RGB-D
camera; the showcase camera is documentary only.

## Static-Obstacle Proof

Run one visible central-barrier trial:

```bash
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 --challenge-obstacles \
  --planner-id RRTConnectkConfigDefault \
  --rviz --gazebo-gui --timeout-s 420 --hold-after-result-s 20 \
  --csv artifacts/runs/obstacle_demo/results.csv \
  --markdown artifacts/runs/obstacle_demo/report.md \
  --log-dir artifacts/runs/obstacle_demo/logs
```

Show that the unchecked Cartesian route is complete while the collision-aware
route is blocked, then show OMPL's alternative transfer and the physical
placement. The report's `direct_path_blocked` field is the auditable evidence;
an RViz trail alone is not a collision proof.

## Safe-Stop Clip

Record the maintained real-wrench profile and its machine-readable verdict:

```bash
bash scripts/run_safe_stop_video.sh artifacts/runs/safe_stop_video
```

The command records the opposite-side Gazebo camera, applies a bounded `16 N`,
`100 ms` wrench during Servo descent stage 3, and only succeeds when the wrench
is applied and cleared, the matching watchdog stops the task, a zero command is
observed within `200 ms`, and the video contains frames. The pick task itself is
expected to end in `FAILED`; that failure is the safe outcome.

## Recording Rules

- Keep the command and result visible at the beginning/end of each clip.
- Record the source commit SHA and experiment configuration beside the video.
- Do not cut away failed attempts from a benchmark batch.
- State that Gazebo ground truth participates in online monitoring and scoring.
- Do not call plan-only animation or attached collision-object motion a physical
  grasp.
- Label the third-person clip as Gazebo physical-contact simulation, not
  real-robot footage.
