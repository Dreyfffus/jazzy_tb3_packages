# Demo Checklist

## Terminal 1: source environment

```bash
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source /ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## Terminal 2: launch Gazebo with bridge

```bash
ros2 launch my_tb3_world sim_with_bridge.launch.py use_sim_time:=true
```

## Terminal 3: launch Nav2

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true params_file:=/ws/src/thermocator/config/nav2_thermal_params.yaml
```

## Terminal 4: launch DT integration

```bash
ros2 launch thermocator dt_integration.launch.py use_sim_time:=true sync_tolerance_seconds:=0.5
```

## Terminal 5: inspect topics

```bash
ros2 topic list | grep /dt
ros2 topic echo /dt/odom --once
ros2 topic echo /dt/scan --once
ros2 topic echo /dt/thermal_reading --once
ros2 topic echo /dt/thermal_map --once
```

## Terminal 6: send a test DT command

```bash
ros2 topic pub --once /dt/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Verify the command is forwarded:

```bash
ros2 topic echo /cmd_vel --once
```

Expected result: `/cmd_vel` receives a `geometry_msgs/msg/TwistStamped` message and the robot moves forward in Gazebo.

## Check sync monitor output

The `dt_integration.launch.py` terminal should print lines like:

```text
Sync ok [odom]
Sync ok [scan]
Sync ok [thermal_map]
```

Warnings indicate the configured tolerance was exceeded.

## Check thermal mapping topics

```bash
ros2 topic echo /thermal_reading --once
ros2 topic echo /thermal_map --once
ros2 topic hz /thermal_reading
ros2 topic hz /thermal_map
```

## Optional RViz safe launch

Use standard TurtleBot3/Nav2/Cartographer RViz displays for map, TF, robot model, laser scan, and goals. Do not add the custom `thermocator/ThermalDisplay` plugin during the safe demo path, because it is a known crash risk.

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```
