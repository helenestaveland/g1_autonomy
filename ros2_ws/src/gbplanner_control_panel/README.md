# GBPlanner RViz panel

This ROS 2 panel provides six GBPlanner controls:
Start Planner, Start Single Planner, Stop Planner, Go Home, Initialization,
and Operation Mode (EXP/WP). The four trigger controls use `std_srvs/Trigger`.
Operation Mode uses `std_srvs/SetBool`; Initialization uses the matching
`planner_msgs` service generated in this workspace. All are listed in
`docker/bridge.yaml`.

Buttons become available when their bridged service appears. The panel has no
internal scroll area and requests enough dock height for all six buttons.
Failed requests and timeouts are logged by RViz in the simulator container.
Initialization is click driven because the upstream
service can arm and take off; the panel never invokes it automatically.

Build with `make build-robot-sim-bringup`, then run `make launch` or
`make launch-uav-sim`. The panel is in the installed simulator RViz layout.
With the simulator alone, its controls remain unavailable until the planner
and bridge are started.
