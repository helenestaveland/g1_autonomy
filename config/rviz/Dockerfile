FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-colcon-common-extensions \
    qtbase5-dev \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-pluginlib \
    ros-humble-std-srvs \
    ros-humble-visualization-msgs \
    ros-humble-grid-map-rviz-plugin \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/g1_ros2_ws

COPY ros2_ws/src/planner_msgs src/planner_msgs
COPY ros2_ws/src/gbplanner_control_panel src/gbplanner_control_panel

RUN source /opt/ros/humble/setup.bash \
    && colcon build --merge-install \
       --packages-select planner_msgs gbplanner_control_panel \
       --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["ros2", "run", "rviz2", "rviz2"]