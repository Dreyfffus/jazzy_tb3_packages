# Thermocator

This is a ROS2 Jazzy package used for temperture mapping and exploration meant for the turtlebot3 burger
model of Opensource robots. It includes multiple nodes meant for map building, navigation and visualization.

## Prerequisite

This project is edited on WSL Ubuntu-24.04 LST, and compiled with colcon on Docker image of osrf/ros:jazzy-desktop-full.


This package needs the full ros2-jazzy installation setup provided with WSL and Docker.
To edit the package you need to also have the minimal installation of ros2-jazzy-base which
provides the headers necessary to develop the application within WSL.

> This can be found on the Module section of Canvas under "Setting up your workspace in WSL.pdf" 
or you can source the included pdf file SETUP.pdf

## Package build instructions

To build the packages do the following:
>[!IMPORTANT]
> Do this to start the docker containing the full jazzy installation:

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
> [!NOTE]
> If you want to also fetch the headers from build to use inside your editor then you have to do this:
```bash
# this exports compile_commands.json to /build folder of the package for import
colcon build [--packages-select thermocator] --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
> This creates a file called `compile_commands.json` somewhere in you build, which is symlinked here.
> If you are using an LSP (***NeoVim***), you need to set up your `.clangd` file that is read for autocompletes
```yaml
CompileFlags:
  CompilationDatabase: /home/c2irr10/turtlebot3_ws/build/thermocator
  Add:
    - "-I/home/c2irr10/turtlebot3_ws/src/thermocator/include"
PathMappings:
  - "/ws/:/home/c2irr10/turtlebot3_ws/"
```
## Running any package

To run any package you source a `*.launch.py` and run the following :

> Specify the package name in ***package name*** and ***launch file***

```bash
 source install/setup.bash
 export TURTLEBOT3_MODEL=burger
 ros2 launch thermocator thermocator.launch.py # not implemented yet
```

Or alternatively you can run any node with :

>  Specify ***node name***. You can find their alias and source file in CMakeLists.txt

```bash
 source install/setup.bash
 export TURTLEBOT3_MODEL=burger
 ros2 run thermocator <node name>
```

## Nodes :

The package has ***2*** nodes :
 
 - `thermocator` is the map building node. It takes in sensor data from **`/thermal_reading`** and publishes to **`/thermal_map`**
 - `thermal_boradcaster` is the mock-sensor input node. It publishes random temperature data to **`/thermal_reading`** and subscribes to it for debug callbacks
>[!IMPORTANT]
> To run these nodes properly, the following commands need to be in order.
```bash
# Terminal 1 - run the world file.

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.properly
```

```bash
# Terminal 2 - run nav2

ros2 launch nav2_bringup navigation2.launch.py use_sim_time:=true # or alternatively SLAM
# ros2 launch slam_toolbox online_async_lanch.py use_sim_time:=true
```

```bash
# Terminal 3 - Run our fake data node 

source install/setup.bash
ros2 launch thermocator thermal_boradcaster
```

```bash
# Terminal 4 - Run thermocator

source install/setup.bash
ros2 run thermocator thermocator
```

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

 >[!TIP]
 > You can greatly speed up the setup by using the `tb3.bash` script. It has multiple functionalities for attaching, starting and
 > running from outside the container useful startup commands.
```bash
# Start the container
./tb3.bash start

# Attach a new terminal to the running container
./tb3.bash attach

# Launch the Gazebo simulation
./tb3.bash remote sim

# Launch Cartographer SLAM + RViz2
./tb3.bash remote rviz

# Launch Nav2 with thermal params
./tb3.bash remote nav

# Run the thermal map builder
./tb3.bash remote thermal

# Run the fake sensor broadcaster
./tb3.bash remote broadcaster

# Teleop keyboard
./tb3.bash remote teleop

# Build everything
./tb3.bash remote build

# Build one package
./tb3.bash remote build thermocator

# Run thermocator on the real robot
./tb3.sh robot thermal 192.168.1.42

# Run broadcaster on the real robot
./tb3.sh robot broadcaster 192.168.1.42

# Run bringup on the real robot
./tb3.sh robot bringup 192.168.1.42
```
