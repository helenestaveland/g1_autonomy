# G1 Humanoid Autonomy

Integration of the Unitree G1 humanoid robot with the Unified Autonomy Stack developed by the NTNU Autonomous Robots Lab (ARL).

The project combines MuJoCo simulation, LiDAR-inertial localization, autonomous path planning, Pure Pursuit path tracking, and a learned locomotion controller.

## System architecture

The system consists of:

- **MuJoCo:** Simulates the Unitree G1 and its environment.
- **Hesai JT128 LiDAR:** Simulated using MuJoCo-LiDAR.
- **MIMOSA (ROS1):** Estimates robot odometry from LiDAR and IMU measurements.
- **GBPlanner (ROS1):** Generates navigation paths.
- **ROS1–ROS2 bridge:** Connects the ROS1 autonomy components with the ROS2 simulation.
- **Pure Pursuit (ROS2):** Converts planned paths into velocity commands.
- **G1 locomotion controller:** Executes velocity commands in the MuJoCo simulation.

### Navigation pipeline

```text
MuJoCo / Unitree G1
    |
    | LiDAR + IMU
    v
ROS1–ROS2 Bridge
    |
    v
MIMOSA
    |
    | Odometry
    v
GBPlanner
    |
    | /gbplanner_path
    v
ROS1–ROS2 Bridge
    |
    v
Pure Pursuit (ROS2)
    |
    | /g1/cmd_vel
    v
G1 Locomotion Controller
    |
    v
MuJoCo / Unitree G1
```

Pure Pursuit is implemented in `scripts/path_tracker_g1.py`, adapted from the original ARL path-tracking implementation.

### Important ROS topics

| Topic | Purpose |
|---|---|
| `/rslidar_points` | Simulated JT128 point cloud |
| `/vectornav_driver_node/imu/data` | Simulated IMU |
| `/mujoco/ground_truth` | Simulation ground truth |
| `/mimosa_node/graph/odometry` | MIMOSA odometry |
| `/gbplanner_path` | Planned navigation path |
| `/g1/cmd_vel` | ROS2 velocity command for G1 |

## Repository structure

- `docker-compose.yml` — Docker services and ROS connections
- `Makefile` — Commands for building and running services
- `docker/g1_sim/` — G1 simulation base image
- `docker/g1_runtime/` — G1 runtime image
- `docker/uas_ros1_base/` — Original ARL ROS1 base image configuration
- `docker/uas_ros1_gbplanner/` — Original ARL GBPlanner image configuration
- `docker/uas_ros1_bridge_builder/` — Original ARL ROS bridge builder
- `manifests/` — Pinned source repository revisions
- `patches/` — Preserved MIMOSA/GTSAM compatibility patches
- `scripts/path_tracker_g1.py` — G1 Pure Pursuit implementation

The original dependencies and project-specific forks are managed through Git manifests rather than duplicated in the main repository.

## Download source repositories

Install Git, Docker and `vcstool` before starting.

From the repository root:

```bash
mkdir -p repos

vcs import repos < manifests/unitree.repos
vcs import repos < manifests/simulation.repos

mkdir -p repos/unified_autonomy_stack/workspaces

vcs import repos/unified_autonomy_stack/workspaces \
  < manifests/robot_bringup.repos

vcs import repos/unified_autonomy_stack/workspaces/ws_mimosa \
  < manifests/ws_mimosa.repos

vcs import repos/unified_autonomy_stack/workspaces/ws_gbplanner \
  < manifests/ws_gbplanner.repos
```

The manifests specify exact repository commits. Detached HEAD states are expected.

## Docker images

### Original NTNU ARL environments

The original Unified Autonomy Stack uses Docker Buildx Bake:

```bash
docker buildx bake --allow=network.host
```

This command belongs to the original ARL Docker build configuration. The G1 repository preserves the relevant ARL Dockerfiles separately, but does not currently reproduce the full original ARL Bake configuration.

The required ARL images are:

- `unified_autonomy:ros1_base`
- `unified_autonomy:ros1_gbplanner`
- `unified_autonomy:ros1-bridge-builder`

Their Dockerfiles are available under `docker/`.

### G1 simulation

The G1 simulation uses project-specific images:

- `g1-sim:dev` — Base environment for MuJoCo and G1 dependencies
- `g1-runtime:local` — Runtime used by MuJoCo, G1 controller and Pure Pursuit

Build the G1 simulation base image first:

```bash
docker build -f docker/g1_sim/Dockerfile -t g1-sim:dev .
```

Then build the runtime image:

```bash
docker compose build mujoco
```

The runtime image depends on `g1-sim:dev`.

## ROS workspace compilation

MIMOSA and GBPlanner must be compiled using Catkin before their ROS launch services can run.

The MIMOSA workspace uses GTSAM-specific CMake settings inherited from the original working environment.

Compatibility patches for GTSAM and MIMOSA are preserved under `patches/`. They are not applied automatically. The existing ARL implementation on MIMOSA branch `dev/agentic_uas`, commit `5ddf925`, should also be evaluated before maintaining a separate compatibility patch.

The original GBPlanner workspace used a Release build with Catkin.


### Build MIMOSA

From the repository root, run:

    docker compose -f docker-compose.build.yml --profile build run --rm build_mimosa

This uses the original ARL ROS1 base image and GTSAM build settings.

### Build GBPlanner

    docker compose -f docker-compose.build.yml --profile build run --rm build_gbplanner

This uses the original ARL GBPlanner image.

Both commands compile the pinned source repositories. Generated build and devel directories remain in their respective workspaces.

The builds and complete G1 integration have not yet been tested.


## Running the system

The Makefile provides the following commands:

```bash
make check
make images
make build
make launch
make launch-unitree
make launch-autonomy
make controller
make status
make logs
make stop
```

`make images` builds the G1 runtime image from its existing base image.

`make build` compiles the G1 controller.

Neither command automatically compiles MIMOSA or GBPlanner. These ROS workspaces must be built separately before starting the complete autonomy system.

The complete reconstructed system has not yet been integration-tested.

## Attribution

This project builds on the original Unified Autonomy Stack from NTNU Autonomous Robots Lab, Unitree Robotics projects, MuJoCo-LiDAR, and their upstream dependencies.

Original source histories and revisions are retained through pinned repositories. G1-specific changes are maintained separately in Git forks, configuration files and project scripts.
