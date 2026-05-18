# Thermocator

A ROS2 Jazzy package for thermal temperature mapping and autonomous exploration, designed for the TurtleBot3 Burger robot. It includes nodes for thermal map building, frontier-based navigation, and an RViz2 visualization plugin.

## Prerequisites

This project is developed on WSL Ubuntu 24.04 LTS and compiled with `colcon` inside a Docker image based on `osrf/ros:jazzy-desktop-full`.

External dependencies:

- ROS2 Jazzy (full desktop)
- RViz2
- Nav2
- Cartographer ROS
- Gazebo with ROS bridge (simulation only)
- TurtleBot3 packages

> [!NOTE]
> Setup instructions are available in `SETUP.pdf` included in this repository, or on Canvas under **"Setting up your workspace in WSL.pdf"**.

---

## Build Instructions

This is an `ament_cmake` package built with `colcon`.

### Linux

Run the following in every terminal before building or running anything:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=38
```

Then build:

```bash
# Build the entire workspace
colcon build

# Build only this package
colcon build --packages-select thermocator

# Build with symlinked install (recommended for fast yaml/config iteration)
colcon build --packages-select thermocator --symlink-install
```

> [!TIP]
> With `--symlink-install`, changes to yaml and config files take effect immediately on the next launch without rebuilding.

### Docker

Start a container with the workspace mounted:

```bash
docker run --rm -it \
  --name turtlebot3_container \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/turtlebot3_ws:/ws \
  turtlebot3_ws bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --packages-select thermocator
source install/setup.bash
```

---

## Running the Package

There are three ways to run the package: via the provided **scripts**, via **launch files**, or **manually** node by node. The launch files and scripts handle the correct startup order automatically.

> [!NOTE]
> The decision node, thermal map builder, and broadcaster will wait for their required topics before doing anything. It is safe to start them before Nav2 or Cartographer are fully up.

---

### Scripts

All scripts are in `scripts/bin/`. They can be run directly or added to your PATH by sourcing `scripts/install/setup.bash`.

#### `robot` — Linux host

Runs ROS2 nodes directly on the host machine. Automatically sources the environment. All paths are resolved relative to the script location.

```bash
robot <service> [domain_id] [package]
```

`domain_id` defaults to `38` if not provided.

| Service | What it runs |
|---|---|
| `teleop` | Keyboard teleoperation |
| `rviz` | Cartographer SLAM + RViz2 |
| `nav` | Nav2 with thermocator params file |
| `lifecycle` | Nav2 lifecycle manager |
| `broadcaster` | Thermal sensor publisher |
| `thermal` | Thermal map builder |
| `decision` | Decision / frontier exploration node |
| `thermocator` | Full thermocator stack (staggered) |
| `thermulator` | Entire system: SLAM + Nav2 + thermocator (staggered) |
| `map_save` | Saves current map to config folder |
| `build` | Builds the workspace. Pass package name as third arg to build selectively |

---

#### `dock` — Docker

Manages a persistent Docker container (`turtlebot3_container`). Starts the container automatically if not already running.

```bash
dock <start | attach | remote> [service] [package]
```

| Command | What it does |
|---|---|
| `start` | Starts a new interactive container session with full environment sourced |
| `attach` | Opens a shell into an already-running container |
| `remote <service>` | Runs a specific service inside the container from the host terminal |

#### `remote` services

| Service | What it runs |
|---|---|
| `teleop` | Keyboard teleoperation |
| `sim` | Gazebo simulation world |
| `rviz` | Cartographer SLAM + RViz2 (`use_sim_time:=True`) |
| `nav` | Nav2 with thermocator params file |
| `lifecycle` | Nav2 lifecycle manager |
| `broadcaster` | Thermal sensor publisher |
| `thermal` | Thermal map builder |
| `decision` | Decision / frontier exploration node |
| `launch` | Full thermocator pipeline (staggered) |
| `build` | Builds the workspace inside the container |

---

### Launch Files

Two launch files are provided:

| File | Purpose |
|---|---|
| `thermocator.launch.py` | Launches only the thermocator nodes (broadcaster, map builder, decision node). Requires Nav2 and Cartographer to already be running. |
| `stack.launch.py` | Launches the entire simulation stack: Cartographer + Nav2 + lifecycle manager + thermocator nodes. Simulation only. |

> [!IMPORTANT]
> Both launch files expose `use_sim_time`. For the real TurtleBot3 on Linux this must be set to `false` — there is no simulation clock. For the Docker simulation setup it must be set to `true`.

#### thermocator.launch.py

```bash
# Load parameters from yaml (default — uses config/thermocator_params.yaml)
ros2 launch thermocator thermocator.launch.py

# Override sim time only
ros2 launch thermocator thermocator.launch.py use_sim_time:=false

# Load a custom params file
ros2 launch thermocator thermocator.launch.py params_file:=/path/to/your_params.yaml

# Override individual arguments (only used if no valid params file is found)
ros2 launch thermocator thermocator.launch.py \
  params_file:=/nonexistent \
  use_sim_time:=true \
  zone_centers_x:=[0.0,1.5] \
  zone_centers_y:=[0.0,-1.0] \
  zone_peak_temps:=[80.0,60.0] \
  zone_sigmas:=[1.2,1.2] \
  cold_threshold:=0.0 \
  hot_threshold:=80.0 \
  heat_detection_threshold:=20.0 \
  scoring_radius:=1.5 \
  max_frontier_distance:=1.0 \
  w_boundary:=1.0 \
  w_thermal_boundary:=3.0 \
  w_hot_interior:=0.5 \
  w_cold_interior:=2.0 \
  revisit_penalty_radius:=0.8 \
  investigation_duration:=5.0 \
  control_rate:=1.0
```

> [!IMPORTANT]
> If `params_file` points to a valid yaml file, it takes full control and individual arguments are ignored (except `use_sim_time` which is always applied). If the file is not found, individual arguments and their defaults are used.

#### stack.launch.py (simulation only)

```bash
ros2 launch thermocator stack.launch.py
```

This launches in the following staggered order:

| Time | What starts |
|---|---|
| 0s | Cartographer SLAM |
| 5s | Nav2 nodes + lifecycle manager |
| 10s | Thermal broadcaster |
| 13s | Thermal map builder |
| 18s | Decision node |

---

### Parameter File

All node parameters can be set in `config/thermocator_params.yaml`. Edit this file to tune behavior without modifying launch files or recompiling.

```yaml
thermal_broadcaster:
  ros__parameters:
    map_frame: "map"
    robot_frame: "base_footprint"
    publish_rate: 2.5
    noise_stdev: 0.5
    zone_centers_x:  [0.0, 1.5]
    zone_centers_y:  [0.0, -1.0]
    zone_peak_temps: [80.0, 60.0]
    zone_sigmas:     [1.2, 1.2]

thermal_map_builder:
  ros__parameters:
    map_frame: "map"
    robot_frame: "base_footprint"
    cold_threshold: 0.0
    hot_threshold:  80.0
    min_confidence: 0.5
    publish_rate:   1.0
    tf_timeout:     0.1

decision_node:
  ros__parameters:
    map_frame:                "map"
    robot_frame:              "base_footprint"
    heat_detection_threshold: 20.0
    frontier_min_distance:    0.8
    scoring_radius:           1.5
    max_frontier_distance:    1.0
    w_boundary:               1.0
    w_thermal_boundary:       3.0
    w_hot_interior:           0.5
    w_cold_interior:          2.0
    revisit_penalty_radius:   0.8
    max_visited_goals:        10
    investigation_duration:   5.0
    control_rate:             1.0
```

> [!NOTE]
> Parameters not present in the yaml file fall back to their hardcoded defaults.

---

### Manual Run

#### Linux — Real Robot

> [!IMPORTANT]
> Run the following at the top of **each** terminal:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source /opt/turtlebot3_ws/install/setup.bash
> export TURTLEBOT3_MODEL=burger
> export ROS_DOMAIN_ID=38
> ```

**Terminal 1 — Robot hardware** *(on the robot's onboard computer)*
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**Terminal 2 — Cartographer SLAM + RViz2**
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=False
```

**Terminal 3 — Nav2**
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=false \
  params_file:=~/turtlebot3_ws/src/thermocator/config/nav2_slam_params.yaml
```

**Terminal 4 — Nav2 Lifecycle Manager**
```bash
ros2 launch thermocator lifecycle_manager.launch.py use_sim_time:=false
```

**Terminal 5 — Thermocator stack**
```bash
ros2 launch thermocator thermocator.launch.py use_sim_time:=false
```

**Terminal 6 — Teleoperation** *(optional)*
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

---

#### Docker — Simulation

> [!IMPORTANT]
> The Docker setup requires an open container. Start one with Terminal 1 below before running any other terminal.

> [!WARNING]
> All commands after Terminal 1 are run from the **host** via `docker exec`. Unless you have added the environment sourcing to `.bashrc` inside the container, every `docker exec` command must source the environment explicitly as shown below.

**Terminal 1 — Start the container**
```bash
docker run --rm -it \
  --name turtlebot3_container \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/turtlebot3_ws:/ws \
  turtlebot3_ws bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /opt/turtlebot3_ws/install/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   exec bash"
```

All following terminals use `docker exec` from the host:

**Terminal 2 — Gazebo simulation**
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 launch my_tb3_world new_world.launch.py"
```

**Terminal 3 — Cartographer SLAM + RViz2**
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"
```

**Terminal 4 — Nav2**
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 launch nav2_bringup navigation_launch.py \
   use_sim_time:=True \
   autostart:=False \
   params_file:=/ws/src/thermocator/config/nav2_slam_params.yaml"
```

**Terminal 5 — Nav2 Lifecycle Manager**
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 launch thermocator lifecycle_manager.launch.py use_sim_time:=true"
```

**Terminal 6 — Thermocator stack**
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 launch thermocator thermocator.launch.py use_sim_time:=true"
```

**Terminal 7 — Teleoperation** *(optional)*
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 run turtlebot3_teleop teleop_keyboard"
```

> [!NOTE]
> For simulation you can launch everything with one command instead of the terminals above:
> ```bash
> docker exec -it turtlebot3_container bash -c \
>   "source /opt/ros/jazzy/setup.bash && \
>    source /ws/install/setup.bash && \
>    export TURTLEBOT3_MODEL=burger && \
>    ros2 launch thermocator stack.launch.py use_sim_time:=true"
> ```

---

## Nodes

### `thermal_broadcaster`

Simulates a thermal sensor by publishing temperature readings based on configurable Gaussian heat zones. Adds configurable noise to simulate real sensor variance. On real hardware this node would be replaced by an actual thermal sensor driver publishing to the same `/thermal_reading` topic.

**Publishes:** `/thermal_reading` (`sensor_msgs/msg/Temperature`)

---

### `thermocator` (thermal_map_builder)

Receives thermal readings from the broadcaster and the occupancy map from Cartographer. For each reading it looks up the robot's position via TF2 and projects the temperature reading onto the corresponding grid cell. Builds a thermal occupancy map over time where cells are scored 0–100 relative to `cold_threshold` and `hot_threshold`. Cells below `min_confidence` are published as unknown (`-1`).

**Subscribes:** `/thermal_reading`, `/map`
**Publishes:** `/thermal_map` (`nav_msgs/msg/OccupancyGrid`, transient local)

---

### `decision_node`

Autonomous thermal exploration brain. Operates in two phases:

**Phase 1 — Spatial exploration:** When no heat has been detected yet, the robot explores unknown space using standard frontier detection (free cells adjacent to unknown cells), moving toward the nearest unexplored boundary.

**Phase 2 — Thermal boundary mapping:** Once heat is detected, the robot shifts to thermal frontier scoring. Candidates are scored by how many unknown cells sit on the boundary between known and unknown thermal space, with a bonus for boundaries adjacent to hot cells. Known cold cells are penalised heavily. Known hot cells are penalised mildly. Candidates too far from any explored area are discarded entirely (`max_frontier_distance`). Recently visited goals are penalised to prevent orbiting.

**Subscribes:** `/map`, `/thermal_map`
**Publishes:** `/thermocator/goal_markers` (`visualization_msgs/msg/MarkerArray`)
**Action client:** `navigate_to_pose` (Nav2)

---

## RViz2 Thermal Display Plugin

The package includes a custom RViz2 plugin that overlays the thermal map on top of the Cartographer map as a color-coded heat visualization.

### Setup

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

In RViz2:

1. Set **Fixed Frame** to `map`
2. **Add** > `RobotModel` > set topic to `/robot_description`
3. **Add** > `Map` > set topic to `/map`
4. **Add** > **By Display Type** > scroll to `thermocator` > select `ThermalDisplay` > set topic to `/thermal_map`

The plugin supports configurable cold and hot colors and transparency, adjustable from the RViz2 Displays panel.

---

## Digital Twin Integration

Use the `dt-integration-support` branch for the digital twin demo:

```bash
git switch dt-integration-support
```

Start the Docker environment with display forwarding:

```bash
./tb3.bash start
```

Inside the container, build the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
cd /ws
colcon build --packages-select my_tb3_world thermocator \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Open new terminals with `./tb3.bash attach` and run:

**Terminal 1 — Gazebo with ROS/Gazebo bridge**
```bash
ros2 launch my_tb3_world sim_with_bridge.launch.py use_sim_time:=true
```

**Terminal 2 — DT mediator, sync monitor, and thermal nodes**
```bash
ros2 launch thermocator dt_integration.launch.py \
  use_sim_time:=true \
  sync_tolerance_seconds:=0.5
```

**Terminal 3 — Nav2 with thermal costmap layer**
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  params_file:=/ws/src/thermocator/config/nav2_thermal_params.yaml
```

### Verification

Check bridge topics are live:
```bash
ros2 topic echo /clock --once
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /tf --once
```

Check digital twin topics:
```bash
ros2 topic list | grep /dt
ros2 topic echo /dt/odom --once
ros2 topic echo /dt/scan --once
ros2 topic echo /dt/thermal_reading --once
ros2 topic echo /dt/thermal_map --once
```

Test command forwarding:
```bash
ros2 topic pub --once /dt/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Expected: `/cmd_vel` receives a `TwistStamped` and the robot moves forward in Gazebo.

Healthy sync output looks like:
```
Sync ok [odom]
Sync ok [scan]
Sync ok [thermal_map]
```

Check thermal pipeline rates:
```bash
ros2 topic hz /thermal_reading
ros2 topic hz /thermal_map
```

For the full demo flow see `docs/demo_checklist.md`.

> [!WARNING]
> Gazebo GUI requires a working display/OpenGL session. In a headless Docker shell `gz sim -g` may fail with a Qt/OpenGL error. Use the full WSLg/X11 Docker session from `./tb3.bash start` for the complete Gazebo movement test.
