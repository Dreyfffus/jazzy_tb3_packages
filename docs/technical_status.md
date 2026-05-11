# Technical Status

## Existing project components

- `thermocator` provides the thermal map builder node. It waits for `/map`, subscribes to `/thermal_reading`, and publishes `/thermal_map`.
- `thermal_broadcaster` provides a simulated thermal sensor stream on `/thermal_reading`.
- `decision_node` can send Nav2 `NavigateToPose` goals based on thermal-map frontier logic.
- `thermocator_layer` is a Nav2 costmap layer plugin intended to inject thermal cost into navigation.
- `thermocator_display` is an RViz display plugin intended to overlay the thermal map.
- `my_tb3_world` provides a custom Gazebo Sim world and a launch file for the TurtleBot3 simulation.
- Helper scripts exist for build, teleop, Gazebo, RViz, Nav2, thermal mapping, and robot-side launch flows.

## Added digital twin support

- `src/my_tb3_world/config/ros_gz_bridge.yaml` defines explicit ROS/Gazebo bridge mappings for `/clock`, `/cmd_vel`, `/odom`, `/scan`, `/tf`, and `/tf_static`.
- `src/my_tb3_world/launch/sim_with_bridge.launch.py` launches the existing custom world and starts `ros_gz_bridge`.
- `dt_mediator` republishes project streams into `/dt/*` topics and forwards `/dt/cmd_vel` to `/cmd_vel`.
- `sync_monitor` compares original topics with `/dt/*` mirror topics and reports header-time or arrival-time delay.
- `src/thermocator/launch/dt_integration.launch.py` starts the thermal pipeline, decision node, DT mediator, and sync monitor together.

## Remaining untested items

- The Nav2 thermal costmap plugin is registered and configured but still needs runtime verification.
- The goal behavior from `decision_node` still needs scenario testing with Nav2 active.
- Gazebo bridge topic names may need adjustment if the active TurtleBot3 Gazebo model publishes namespaced Gazebo topics.
- `/dt/cmd_vel -> /cmd_vel` forwarding is implemented. The DT input uses `geometry_msgs/msg/Twist`; the mediator republishes `geometry_msgs/msg/TwistStamped` on `/cmd_vel` for Jazzy/TurtleBot3/Nav2 compatibility. Bidirectional motion must be verified with the running simulation.
- End-to-end timing tolerance must be measured during a live demo run.

## Known risks intentionally not fixed here

- The custom RViz thermal display plugin is known to crash visualization. This pass does not modify or fix it.
- Thermal grid math/indexing issues were previously identified. This pass does not modify or fix them.
- The Nav2 thermal costmap plugin is untested and may need parameter or lifecycle adjustments after runtime testing.
- The Gazebo world launch remains unchanged, so any existing world/spawn behavior is preserved.

## Gazebo as the digital twin visualization

Gazebo Sim is used as the digital twin visualization because it presents the robot, environment geometry, obstacles, sensor simulation, and simulated motion in one synchronized world. RViz remains useful for ROS introspection, maps, TF, Nav2 plans, and debugging, but Gazebo is the primary spatial visualization of the twin environment. The `/dt/*` topics provide a stable digital-twin-facing interface that mirrors robot, sensor, map, and thermal state without changing the teammate implementation.
