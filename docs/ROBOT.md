# Robot Launch Commands

>Every command needs the same previous line
>Must be done from working directory : turtlebot3_ws
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```
From here we can run ros2 executables and package nodes with `ros2 run`and `ros2 launch`

## Actions

> Launch Robot
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
> Launch RViz through turtlebot ssh
```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```
> Start Teleop on Physical Robot
```bash
# "/cmd_vel:=/cmd_vel_raw" tells teleop to publish to channel /cmd_vel 
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/cmd_vel_raw
```


