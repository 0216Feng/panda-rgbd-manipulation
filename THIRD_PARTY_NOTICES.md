# Third-Party Notices

## MoveIt Resources Panda description

`src/panda_manipulation/urdf/panda_balanced.urdf.xacro` is derived from the
Franka Panda description distributed by the MoveIt Resources project:

- Upstream repository: https://github.com/moveit/moveit_resources
- Upstream package: `panda_description`
- Upstream branch used by the ROS2 package: `ros2`
- License: Apache License 2.0

The local derivative changes the right-finger inertial mass to match the left
finger. It is loaded by this project's simulation/control integration. The file
retains its origin notice and carries a prominent modification note.

The Apache-2.0 text is included at
[`LICENSES/Apache-2.0.txt`](LICENSES/Apache-2.0.txt). Meshes are not copied into
this repository; the Xacro references the separately installed
`moveit_resources_panda_description` package at runtime.

ROS2, MoveIt2, Gazebo, OpenCV, OMPL, and other installed dependencies keep their
respective upstream licenses. They are dependencies, not relicensed by this
project's MIT license.
