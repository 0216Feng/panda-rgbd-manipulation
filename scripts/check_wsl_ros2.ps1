$ErrorActionPreference = "Stop"

Write-Host "WSL distributions:"
wsl --list --verbose

Write-Host "`nWSL ROS2 check:"
wsl bash -lc @'
set -e
echo "Ubuntu:"
lsb_release -a 2>/dev/null || cat /etc/os-release
echo
echo "ROS:"
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
command -v ros2 || true
command -v colcon || true
command -v rosdep || true
echo
echo "Key packages:"
dpkg -l | grep -E 'ros-jazzy-(moveit|moveit-resources-panda|ros-gz|ros2-control|ros2-controllers|cv-bridge)' || true
'@
