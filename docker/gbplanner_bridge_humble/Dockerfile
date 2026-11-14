# syntax=docker/dockerfile:1

FROM unified_autonomy:ros1-bridge-builder

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]

ARG ROS1_BRIDGE_COMMIT=8f1b7e903e1fb4462218db659bb4981228c12206

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    ros-humble-grid-map-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY --from=bridge_ros1_planner . \
    /g1_interfaces/ros1_ws/src/planner_msgs

COPY --from=bridge_ros1_gridmap . \
    /g1_interfaces/ros1_ws/src/grid_map_msgs

COPY --from=bridge_ros2_src . \
    /g1_interfaces/ros2_ws/src

COPY ros1_bridge_bidirectional_qos.patch /tmp/bridge.patch

RUN unset ROS_DISTRO \
    && cd /g1_interfaces/ros1_ws \
    && colcon build --merge-install \
       --packages-select planner_msgs grid_map_msgs \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=Release \
         -DPYTHON_EXECUTABLE=/usr/bin/python3

RUN source /opt/ros/humble/setup.bash \
    && cd /g1_interfaces/ros2_ws \
    && colcon build --merge-install \
       --packages-select planner_msgs g1_bridge_mappings \
       --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN mkdir -p /ws_g1_bridge/src \
    && git init /ws_g1_bridge/src/ros1_bridge \
    && cd /ws_g1_bridge/src/ros1_bridge \
    && git remote add origin https://github.com/smith-doug/ros1_bridge.git \
    && git fetch --depth=1 origin "${ROS1_BRIDGE_COMMIT}" \
    && git checkout --detach FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${ROS1_BRIDGE_COMMIT}" \
    && git apply --check /tmp/bridge.patch \
    && git apply /tmp/bridge.patch

RUN source /opt/ros/humble/setup.bash \
    && source /g1_interfaces/ros1_ws/install/local_setup.bash \
    && source /g1_interfaces/ros2_ws/install/local_setup.bash \
    && cd /ws_g1_bridge \
    && MAKEFLAGS="-j2" colcon build \
       --parallel-workers 2 \
       --packages-select ros1_bridge \
       --cmake-args \
         -DBUILD_TESTING=OFF \
         -DCMAKE_BUILD_TYPE=Release \
    && source install/local_setup.bash \
    && ros2 run ros1_bridge dynamic_bridge --print-pairs \
       > /tmp/bridge-pairs.txt \
    && grep -Fq "'grid_map_msgs/msg/GridMap'" /tmp/bridge-pairs.txt \
    && grep -Fq "'planner_msgs/srv/PciInitialization'" /tmp/bridge-pairs.txt \
    && grep -Fq "'planner_msgs/srv/PciGlobal'" /tmp/bridge-pairs.txt

CMD ["bash"]