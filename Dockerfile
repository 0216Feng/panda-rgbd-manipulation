FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-pip \
    python3-rosdep \
    python3-pytest \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-cv-bridge \
    ros-jazzy-moveit \
    ros-jazzy-moveit-resources-panda-description \
    ros-jazzy-moveit-resources-panda-moveit-config \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY . /workspace

RUN source /opt/ros/jazzy/setup.bash && \
    if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update --rosdistro jazzy && \
    rosdep install --from-paths /workspace/src --ignore-src --rosdistro jazzy -y

CMD ["bash"]
