# Gazebo Video Evidence

This directory binds the public Gazebo third-person video to the physical
benchmark result that produced it. The run used markerless RGB-D target
localization, the `center_tall_barrier` challenge, RRTConnect, physical finger
control, loaded-path precheck, transfer diagnostics, and independent Gazebo
outcome validation.

- Date: 2026-09-12
- Pipeline and validator: `SUCCESS`
- Maximum lift: `0.4567 m`
- Placement error: `0.0115 m`
- Final tilt: `0.00 deg`
- Direct collision-aware path blocked: `true`
- RGB-D mean target error: `0.0011 m`
- Published video: `docs/assets/demo/gazebo_obstacle_pick.mp4`
- Published video SHA256: `763661e421d616fdbe2b33dcd4e8f8edc19766d72b541ae22267d4ee719d971d`

The video is Gazebo physical-contact simulation evidence. It is not real-robot
footage. `results.csv` is the quantitative source of truth; the Markdown and
SVG files are derived views, and the JSON sidecars record configuration and
source fingerprints.
