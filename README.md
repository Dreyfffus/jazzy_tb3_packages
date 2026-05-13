# Thermocator

This is a ROS2 Jazzy package used for temperture mapping and exploration meant for the turtlebot3 burger
model of Opensource robots. It includes multiple nodes meant for map building, navigation and visualization.

## Prerequisite

This project is edited on WSL Ubuntu-24.04 LST, and compiled with colcon on Docker image of osrf/ros:jazzy-desktop-full.

This package depends on external packages like : RViz2, Nav2, Ros Cartographer, Rosbridge for Gazebo and the standard 
Jazzy Ros2 environment.

> This can be found on the Module section of Canvas under "Setting up your workspace in WSL.pdf" 
or you can source the included pdf file SETUP.pdf

## Package build instructions

Colcon is the package builder used for most ros2 packages. Obviously, colcon is used 
int he building of this package. The package is structured using the standard package 
template provided by Ros. This is an `ament_cmake` package, meaning the package build file
`CmakeLists.txt` is available at the root of the folder. Additionally, this package is mostly C++.

### Linux Setup:

Just go to the Repository Root directory and type :
```bash
 source /opt/ros/jazzy/setup.bash
 source /opt/turtlebot3_ws/install/setup.bash

# This is for building the entire package
colcon build

# Also modules can be build separately:
colcon build --packages-select <thermocator | thermal_boradcaster>
```

### Docker Setup:

If you are using the provided `SETUP.pdf` as a guide to the environment, then
to build the packages do the following:

>[!IMPORTANT]
> For manual build you need to first start a ros-jazzy container.
> The following command can be run to do so. The `-rm` flag tells docker to destroy the container upon exit. 
> This is fine, since you dock the workdir onto `/ws` upon startup. 

```bash
 docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY -v \
    /tmp/.X11-unix:/tmp/.X11-unix -v /home/c2irr10/turtlebot3_ws:/ws turtlebot3_ws bash
```

> After this step, you can run :

```bash
# Installs the specified dependencies of packages
 rosedep install -i --from-path src --rosdistro jazzy -y #you have to be in the package directory 

# Provides the environment for running ros2 commands
 source /opt/ros/jazzy/setup.bash
 source /opt/turtlebot3_ws/install/setup.bash

# This builds the source files of the package
 cd src/thermocator && colcon build #or 
 colcon build --packages-select thermocator
```

## Running the package

This package has multiple modules that interact with each other. There are 3 main ways
to run the entire package with full functionality. One is by using the scripts provided
in the `scripts/` directory. The other is by **launching** the services. Third is by
**running** each node manually:

>[!NOTE]
> The launch files inside tha package have the entire setup ready made to be run from one single
> entry point. Additionally the scripts, which are segregated for both setups, are also available to run
> nodes individually, speeding up this process. The manual run is just an explanation of what is happening
> in the provided tools.

### Manual Run:

---

#### Linux

>[!IMPORTANT]
> Every terminal needs the environment sourced first. Run this at the top of **each** terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=38
```

---

> **Terminal 1** — Cartographer SLAM + RViz2
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

> **Terminal 2** — Nav2
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=False \
  autostart:=False \
  params_file:=~/turtlebot3_ws/src/thermocator/config/nav2_slam_params.yaml
```

> **Terminal 3** — Nav2 Lifecycle Manager
```bash
ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args \
  -p use_sim_time:=false \
  -p autostart:=true \
  -p node_names:='["controller_server","smoother_server","planner_server", \ 
  "behavior_server","velocity_smoother","collision_monitor","bt_navigator","waypoint_follower"]'
```

> **Terminal 4** — Thermal Broadcaster
```bash
ros2 run thermocator thermal_broadcaster
```

> **Terminal 5** — Thermal Map Builder
```bash
ros2 run thermocator thermocator
```

> **Terminal 6** — Decision Node
```bash
ros2 run thermocator decision_node
```

> **Terminal 7** — Teleoperation _(optional)_
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

---

#### Docker

>[!IMPORTANT] 
> The docker setup has to have an opened container.
> Thus, the first step is opening one and exporting everything.

>**Terminal 1** — Start the container _(once)_
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

For all following terminals, each command is run **from the host** via `docker exec`:

---
>[!WARNING] 
> Unfortunately, the way you run these files manually is by remotely attaching to the session
> and running the scripts using `docker exec`. This means that unless you place the setup for every command
> inside `.bashrc` you will have to run everything like its seen below.

> **Terminal 2** — Gazebo Simulation
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /opt/turtlebot3_ws/install/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 launch my_tb3_world new_world.launch.py"
```

> **Terminal 3** — Cartographer SLAM + RViz2
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /opt/turtlebot3_ws/install/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"
```

> **Terminal 4** — Nav2
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /opt/turtlebot3_ws/install/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 launch nav2_bringup navigation_launch.py \
   use_sim_time:=True \
   autostart:=False \
   params_file:=/ws/src/thermocator/config/nav2_slam_params.yaml"
```

> **Terminal 5** — Nav2 Lifecycle Manager
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /opt/turtlebot3_ws/install/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 run nav2_lifecycle_manager lifecycle_manager \
   --ros-args \
   -p use_sim_time:=false \
   -p autostart:=true \
   -p node_names:='[\"controller_server\",\"smoother_server\",\"planner_server\",\"behavior_server\",\"velocity_smoother\",\"collision_monitor\",\"bt_navigator\",\"waypoint_follower\"]'"
```

> **Terminal 6** — Thermal Broadcaster
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /opt/turtlebot3_ws/install/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 run thermocator thermal_broadcaster \
   --ros-args -p use_sim_time:=true"
```

> **Terminal 7** — Thermal Map Builder
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /opt/turtlebot3_ws/install/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 run thermocator thermocator \
   --ros-args -p use_sim_time:=true"
```

> **Terminal 8** — Decision Node
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /opt/turtlebot3_ws/install/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 run thermocator decision_node \
   --ros-args -p use_sim_time:=true"
```

> **Terminal 9** — Teleoperation _(optional)_
```bash
docker exec -it turtlebot3_container bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /opt/turtlebot3_ws/install/setup.bash && \
   source /ws/install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   ros2 run turtlebot3_teleop teleop_keyboard"
```
### Launching 

Alternatively, you can **launch** the package using the provided launch files.
These describe the exact running sequence of every node. Two of these are provided:

* **thermocator.launch.py** -- launches the bare package stack without any dependencies.
* **thermulator.launch.py** -- launches the entire stack of the package. 

```bash
# For the bare stack, needs setup ran in the proper order.
# Running the package before the setup is fine, all nodes freeze until topics exist and publish.
ros2 launch thermocator thermocator.launch.py 

# Launches full stack
ros2 launch thermocator thermocator.launch.py \
  use_sim_time:=true \          # Use Gazebo clock instead of wall clock
  map_frame:=map \              # TF frame name for the map
  robot_frame:=base_footprint \ # TF frame name for the robot base
  zone_centers_x:=[0.0,1.5] \  # X positions of simulated heat zones in map frame (meters)
  zone_centers_y:=[0.0,-1.0] \ # Y positions of simulated heat zones in map frame (meters)
  zone_peak_temps:=[80.0,60.0] \ # Peak temperature of each zone (°C)
  zone_sigmas:=[1.2,1.2] \     # Gaussian spread of each heat zone (meters) — larger = wider zone
  cold_threshold:=0.0 \        # Temperature mapped to occupancy 0 (coldest expected value)
  hot_threshold:=80.0 \        # Temperature mapped to occupancy 100 (hottest expected value)
  min_confidence:=0.5 \        # Minimum reading confidence for a cell to be published (0.0–1.0)
  publish_rate:=1.0 \          # How often the thermal map is published (Hz)
  heat_detection_threshold:=20.0 \ # Occupancy value above which a cell is considered hot (0–100)
  scoring_radius:=1.5 \        # Radius around each frontier candidate used for heat scoring (meters)
  w_unknown_hot:=3.0 \         # Weight bonus for unknown cells adjacent to hot cells
  w_dist_hottest:=0.5 \        # Penalty weight for being far from the hottest known cell
  w_cold_penalty:=0.2 \        # Penalty weight for confirmed cold cells within scoring radius
  frontier_min_distance:=0.8 \ # Minimum distance frontier -> robot to be considered (meters)
  investigation_duration:=5.0 \ # Seconds the robot waits at a goal before rescanning
  control_rate:=1.0             # How often the decision node runs its control loop (Hz)
```

>[!IMPORTANT] 
> The launch files both expose `use_sim_time`.
> For the Turtlebot3 -- Linux setup this must be set to false, as there is no simulation clock.
> Aversely, for the Docker setup `use_sim_time` must be set to true explicitly for the packages 
> to work properly.

### Scripts 

All scripts provided in the **`scripts/bin/`** folder can either be ran as is or they can be introduced
into the path by sourcing **`scripts/install/setup.bash`**. This allows you to run the script as if it were 
a binary file.

#### `robot` -- Linux

Runs ROS 2 nodes directly on the host machine. Sources the ROS 2 Jazzy and workspace environments automatically, then runs the requested service. All paths are resolved relative to the script's location so it works regardless of where the workspace is cloned.

```bash
robot <service> [domain_id] [package]
```

`domain_id` defaults to `38` if not provided.

| Service | What it runs |
|---------|-------------|
| `teleop` | Keyboard teleoperation node |
| `rviz` | Cartographer SLAM + RViz2 |
| `nav` | Nav2 navigation stack with the thermocator params file |
| `lifecycle` | Nav2 lifecycle manager, activating all navigation nodes |
| `broadcaster` | Simulated thermal sensor publisher |
| `thermal` | Thermal map builder node |
| `decision` | Frontier exploration / decision node |
| `thermocator` | Launches the full thermocator stack (broadcaster → map builder → decision node, staggered) |
| `thermulator` | Launches the entire system (SLAM + Nav2 + thermocator stack, staggered) |
| `map_save` | Saves the current map to the config folder |
| `build` | Builds the workspace. Pass a package name as the third argument to build only that package |

---

#### `dock` -- Docker

Manages a persistent Docker container (`turtlebot3_container`) running the `turtlebot3_ws` image. If the container is not already running, it is started automatically before executing any `remote` command.

```bash
dock <start | attach | remote> [service] [package]
```

| Command | What it does |
|---------|-------------|
| `start` | Starts a new interactive container session with the full environment sourced |
| `attach` | Opens a shell into an already-running container (starts it first if needed) |
| `remote <service>` | Runs a specific service inside the container from the host terminal |

#### `remote` -- services

| Service | What it runs |
|---------|-------------|
| `teleop` | Keyboard teleoperation node |
| `rviz` | Cartographer SLAM + RViz2 (`use_sim_time:=True`) |
| `sim` | Gazebo simulation world |
| `nav` | Nav2 navigation stack with the thermocator params file |
| `lifecycle` | Nav2 lifecycle manager, activating all navigation nodes |
| `broadcaster` | Simulated thermal sensor publisher |
| `thermal` | Thermal map builder node |
| `decision` | Frontier exploration / decision node |
| `launch` | Full thermocator pipeline (broadcaster → map builder → decision node, staggered) |
| `build` | Builds the workspace inside the container. Pass a package name as the third argument to build only that package |


## Digital twin integration test

Use the `dt-integration-support` branch for the digital twin demo.

```bash
git switch dt-integration-support
```

Start the Docker environment with display forwarding, then build the workspace:

```bash
./tb3.bash start
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
cd /ws
colcon build --packages-select my_tb3_world thermocator --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Open new terminals with `./tb3.bash attach`, source the same environment, and run:

```bash
# Terminal 1: Gazebo with ROS/Gazebo bridge
ros2 launch my_tb3_world sim_with_bridge.launch.py use_sim_time:=true
```

```bash
# Terminal 2: DT mediator, sync monitor, and thermal nodes
ros2 launch thermocator dt_integration.launch.py use_sim_time:=true sync_tolerance_seconds:=0.5
```

```bash
# Terminal 3: Nav2 with the thermal costmap layer
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  params_file:=/ws/src/thermocator/config/nav2_thermal_params.yaml
```

Check the Gazebo bridge topics:

```bash
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /tf --once
```

Check the digital twin topics:

```bash
ros2 topic list | grep /dt
ros2 topic echo /dt/odom --once
ros2 topic echo /dt/scan --once
ros2 topic echo /dt/thermal_reading --once
ros2 topic echo /dt/thermal_map --once
```

Test command forwarding from the digital twin interface:

```bash
ros2 topic echo /cmd_vel --once
```

In another terminal:

```bash
ros2 topic pub --once /dt/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Expected result: `/cmd_vel` receives a `geometry_msgs/msg/TwistStamped` command and the robot moves forward in Gazebo.

Check synchronization output in the `dt_integration.launch.py` terminal. Healthy streams print lines like:

```text
Sync ok [odom]
Sync ok [scan]
Sync ok [thermal_map]
```

Check the thermal pipeline:

```bash
ros2 topic echo /thermal_reading --once
ros2 topic echo /thermal_map --once
ros2 topic hz /thermal_reading
ros2 topic hz /thermal_map
```

For the full manual demo flow, follow `docs/demo_checklist.md`.

> Note: Gazebo GUI needs a working display/OpenGL session. In a headless Docker shell, `gz sim -g` can fail with a Qt/OpenGL error. Use the normal WSLg/X11 Docker session from `./tb3.bash start` for the full Gazebo movement test.

## Nodes :

**`thermal_broadcaster`**
Simulates a thermal sensor. Publishes fake temperature readings based on Gaussian heat zones defined in the launch arguments (`zone_centers_x/y`, `zone_peak_temps`, `zone_sigmas`). Adds configurable noise. This is the data source for the thermal map builder — on real hardware this would be replaced by an actual sensor driver.
 
**`thermocator` (thermal_map_builder)**
Subscribes to thermal readings from the broadcaster and the occupancy map from Cartographer. For each reading it looks up the robot's position in the map frame via TF and projects the temperature onto the corresponding map cell. Over time it builds a thermal occupancy map — cells are scored 0–100 based on how hot they are relative to `cold_threshold` and `hot_threshold`. Only cells with sufficient confidence are published.
 
**`decision_node`**
The autonomous exploration brain. Reads the occupancy map from Nav2 and the thermal map from the map builder to identify frontiers (boundaries between known and unknown space). Scores each frontier by how much thermal activity is nearby, how close it is to the hottest known cell, and how many cold cells are in its vicinity. Sends the highest-scoring frontier as a navigation goal to Nav2, waits at the goal for `investigation_duration` seconds, then rescans and picks the next target.
 

## Rviz2

The Package includes an **rviz2 visualization plugin** of the mapped temperature map. This is an overlay in rviz2 that needs manual insertion. 
> To use this node you need rviz2 and ros cartographer. The following instructions explain use case.
```bash
# Run the cartographer interface

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```
 - Set Fixed Frame to `map`
 - Add the TurtleBot3 Model : **Add** > `RobotModel` > **topic** > `/robot_description`
 - Add the SLAM map : **Add** > `Map` > **topic** > `/map`
 - Add the thermal layer :  **Add** > **By Display Type** > scroll to `thermocator` > **ThermalDisplay** > set topic to `/thermal_map`
