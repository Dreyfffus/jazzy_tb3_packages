# ROS Bridge and DT Integration

## What this project is doing

This repository builds a ROS 2 Jazzy workflow around a TurtleBot3 digital twin demo. At a high level, it combines:

- a Gazebo Sim world for the robot and environment,
- standard ROS 2 navigation and mapping topics such as `/scan`, `/odom`, `/tf`, and `/map`,
- a simulated thermal sensing pipeline,
- a digital-twin-facing topic interface under `/dt/*`.

The main runtime pieces are:

- `my_tb3_world`: launches the custom Gazebo world and the Gazebo/ROS bridge.
- `thermal_broadcaster`: simulates a thermal sensor reading based on the robot pose in the map.
- `thermocator`: builds a thermal occupancy-grid map from the base map and thermal readings.
- `decision_node`: chooses exploration goals from the thermal and spatial maps.
- `dt_mediator`: exposes project state to the digital twin interface and accepts DT-side commands.
- `sync_monitor`: checks whether mirrored `/dt/*` topics stay in sync with the original ROS topics.

## Branch note

This local checkout is currently on `main`, not a local branch named `dt-integration`. The DT work visible here appears to come from the merged remote branch `origin/dt-integration-support`, including:

- `src/my_tb3_world/launch/sim_with_bridge.launch.py`
- `src/my_tb3_world/config/ros_gz_bridge.yaml`
- `src/thermocator/src/dt_mediator.cpp`
- `src/thermocator/src/sync_monitor.cpp`
- `src/thermocator/launch/dt_integration.launch.py`

## The most important clarification: there are two different "bridge" layers

People can easily say "the rosbridge thing" and mean different parts of this repo. In this project there are two separate bridging concepts:

1. `ros_gz_bridge`
   This is the actual Gazebo-to-ROS 2 bridge. It converts Gazebo transport messages into ROS 2 topics and converts ROS 2 commands back into Gazebo message types.

2. `dt_mediator`
   This is not Gazebo bridging. It is the digital twin adapter inside the ROS graph. It mirrors selected ROS topics into `/dt/*` topics and forwards DT control commands back to the robot command topic.

If someone asks "how does rosbridge work here?", they usually mean `ros_gz_bridge`, but the full DT story only makes sense when both layers are described together.

## What `ros_gz_bridge` does

`ros_gz_bridge` is launched by:

- `src/my_tb3_world/launch/sim_with_bridge.launch.py`

That launch file starts:

- the custom Gazebo world from `new_world.launch.py`
- a `ros_gz_bridge` node running the `parameter_bridge` executable

The bridge configuration lives in:

- `src/my_tb3_world/config/ros_gz_bridge.yaml`

That YAML file explicitly maps topic names and message types between Gazebo Sim and ROS 2:

- `/clock`: Gazebo clock into ROS 2
- `/cmd_vel`: ROS 2 velocity command into Gazebo
- `/odom`: Gazebo odometry into ROS 2
- `/scan`: Gazebo laser scan into ROS 2
- `/tf`: Gazebo poses into ROS 2 TF
- `/tf_static`: Gazebo static poses into ROS 2 TF

## How `ros_gz_bridge` works

The bridge works as a type-conversion relay:

1. Gazebo publishes on its own transport system using Gazebo message types such as `gz.msgs.Odometry` and `gz.msgs.LaserScan`.
2. `ros_gz_bridge` subscribes to those Gazebo topics.
3. It converts each incoming Gazebo message to the configured ROS 2 type.
4. It republishes the converted message into the ROS 2 graph under the configured ROS topic name.

For commands in the other direction:

1. A ROS 2 node publishes to `/cmd_vel`.
2. `ros_gz_bridge` subscribes to that ROS 2 topic.
3. It converts the ROS 2 message into the Gazebo message type.
4. Gazebo receives that command and applies it in simulation.

In this repo the direction is explicit in the YAML:

- `GZ_TO_ROS` for `/clock`, `/odom`, `/scan`, `/tf`, `/tf_static`
- `ROS_TO_GZ` for `/cmd_vel`

So the bridge is the reason Gazebo sensor data becomes visible to normal ROS 2 nodes, and also the reason a ROS 2 velocity command can move the simulated TurtleBot3.

## Why `/cmd_vel` uses `TwistStamped`

The bridge config maps ROS `/cmd_vel` as:

- `geometry_msgs/msg/TwistStamped` on the ROS side
- `gz.msgs.Twist` on the Gazebo side

That matters because DT-side publishers do not write directly to Gazebo. They publish a plain `geometry_msgs/msg/Twist` to `/dt/cmd_vel`, and then `dt_mediator` upgrades it to `TwistStamped` before publishing to `/cmd_vel`.

This means the command path is:

`/dt/cmd_vel` as `Twist`
-> `dt_mediator`
-> `/cmd_vel` as `TwistStamped`
-> `ros_gz_bridge`
-> Gazebo `gz.msgs.Twist`

## What `dt_mediator` does

`dt_mediator` is implemented in:

- `src/thermocator/src/dt_mediator.cpp`

It has two jobs.

### 1. Mirror ROS topics into DT topics

It republishes:

- `/odom` -> `/dt/odom`
- `/scan` -> `/dt/scan`
- `/map` -> `/dt/map`
- `/thermal_map` -> `/dt/thermal_map`
- `/thermal_reading` -> `/dt/thermal_reading`

This gives a digital twin consumer a stable interface without needing to subscribe directly to the original project topics.

### 2. Forward DT commands back to the robot

It subscribes to:

- `/dt/cmd_vel` as `geometry_msgs/msg/Twist`

It then creates a new `geometry_msgs/msg/TwistStamped` message:

- `header.stamp = now()`
- `header.frame_id = "base_link"`
- `twist = incoming /dt/cmd_vel payload`

Finally it publishes that stamped command to:

- `/cmd_vel`

This is the compatibility step that lets a DT-facing controller remain simple while still satisfying the ROS/Gazebo command interface expected by the rest of the stack.

## What `sync_monitor` does

`sync_monitor` is implemented in:

- `src/thermocator/src/sync_monitor.cpp`

It does not bridge messages. It verifies the health of the DT mirror.

It watches original and mirrored pairs such as:

- `/odom` and `/dt/odom`
- `/scan` and `/dt/scan`
- `/thermal_map` and `/dt/thermal_map`

For each pair it compares:

- header timestamp difference when both messages have valid header stamps,
- otherwise arrival-time difference inside the node.

If the delay is under the configured tolerance, it logs `Sync ok`.
If the delay exceeds the threshold, it logs `Sync warning`.

So `sync_monitor` is the observability layer for the DT interface.

## End-to-end data flow

### Sensor and state path

1. Gazebo simulates the robot and publishes `/odom`, `/scan`, `/tf`, and `/clock` on Gazebo transport.
2. `ros_gz_bridge` converts those into ROS 2 topics.
3. The thermal pipeline consumes those ROS topics.
4. `dt_mediator` mirrors the resulting robot and map state to `/dt/*`.
5. A digital twin client can observe `/dt/odom`, `/dt/scan`, `/dt/map`, `/dt/thermal_map`, and `/dt/thermal_reading`.

### Thermal path

1. `thermal_broadcaster` uses TF to estimate the robot position in the map frame.
2. It computes a simulated temperature from configured heat zones.
3. It publishes `/thermal_reading`.
4. `thermocator` combines `/map` and `/thermal_reading` into `/thermal_map`.
5. `dt_mediator` mirrors both thermal topics into the `/dt/*` namespace.

### Command path

1. A DT-side tool publishes `geometry_msgs/msg/Twist` to `/dt/cmd_vel`.
2. `dt_mediator` converts it to `geometry_msgs/msg/TwistStamped` and republishes on `/cmd_vel`.
3. `ros_gz_bridge` converts `/cmd_vel` into Gazebo's `gz.msgs.Twist`.
4. Gazebo applies the motion to the TurtleBot3.
5. The resulting odometry and scan updates come back through the bridge and appear again in ROS and `/dt/*`.

This closes the loop between DT control, simulation motion, and mirrored DT state.

## Launch files and responsibilities

### `ros2 launch my_tb3_world sim_with_bridge.launch.py`

Starts:

- Gazebo world
- TurtleBot3 spawn
- robot state publisher
- `ros_gz_bridge`

Use this when you need the simulation and ROS/Gazebo topic conversion.

### `ros2 launch thermocator dt_integration.launch.py`

Starts:

- `thermal_broadcaster`
- `thermocator`
- `decision_node`
- `dt_mediator`
- `sync_monitor`

Use this when you need the thermal pipeline and DT mirror interface inside ROS 2.

In practice, the two launch files are complementary:

- `sim_with_bridge.launch.py` makes Gazebo data available to ROS 2.
- `dt_integration.launch.py` makes project data available to the digital twin interface.

## Topic ownership cheat sheet

### Produced by Gazebo and bridged into ROS

- `/clock`
- `/odom`
- `/scan`
- `/tf`
- `/tf_static`

### Produced inside the ROS project

- `/thermal_reading`
- `/thermal_map`
- `/map`

### Produced for the digital twin interface

- `/dt/odom`
- `/dt/scan`
- `/dt/map`
- `/dt/thermal_map`
- `/dt/thermal_reading`

### Consumed from the digital twin interface

- `/dt/cmd_vel`

## How to test this part

The easiest way to test the DT integration is to verify the system in three layers:

1. `ros_gz_bridge` is alive and Gazebo topics reach ROS 2.
2. `dt_mediator` is mirroring project topics into `/dt/*`.
3. DT-side commands sent to `/dt/cmd_vel` reach `/cmd_vel` and move the robot in Gazebo.

### Recommended launch sequence

Build and source the workspace first, then use separate terminals.

Terminal 1:

```bash
ros2 launch my_tb3_world sim_with_bridge.launch.py use_sim_time:=true
```

Terminal 2:

```bash
ros2 launch thermocator dt_integration.launch.py use_sim_time:=true sync_tolerance_seconds:=0.5
```

Optional Terminal 3 if you also want Nav2 active:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  params_file:=/ws/src/thermocator/config/nav2_thermal_params.yaml
```

### Test 1: verify the Gazebo <-> ROS bridge

In a new terminal:

```bash
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /odom --once
ros2 topic echo /scan --once
```

Pass condition:

- `/clock` is publishing.
- `/odom` and `/scan` produce messages while Gazebo is running.

If this fails, the problem is usually in `ros_gz_bridge` or the Gazebo side, not `dt_mediator`.

### Test 2: verify DT mirror topics exist

In a new terminal:

```bash
ros2 topic list | grep /dt
```

You should see at least:

- `/dt/odom`
- `/dt/scan`
- `/dt/map`
- `/dt/thermal_map`
- `/dt/thermal_reading`
- `/dt/cmd_vel`

Then probe a few mirrored streams:

```bash
ros2 topic echo /dt/odom --once
ros2 topic echo /dt/scan --once
ros2 topic echo /dt/thermal_reading --once
ros2 topic echo /dt/thermal_map --once
```

Pass condition:

- the `/dt/*` topics exist,
- they receive data when the source ROS topics are active.

### Test 3: verify command forwarding

First watch the real robot command topic:

```bash
ros2 topic echo /cmd_vel --once
```

In another terminal publish a DT-side command:

```bash
ros2 topic pub --once /dt/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Pass condition:

- `/cmd_vel` receives a forwarded message,
- the message type on `/cmd_vel` is `geometry_msgs/msg/TwistStamped`,
- the robot moves forward in Gazebo.

This specifically proves:

- `dt_mediator` accepted `/dt/cmd_vel`,
- it converted `Twist` into `TwistStamped`,
- `ros_gz_bridge` forwarded the ROS command into Gazebo.

### Test 4: verify sync health

Watch the terminal running `dt_integration.launch.py`.

Healthy streams should print messages like:

```text
Sync ok [odom]
Sync ok [scan]
Sync ok [thermal_map]
```

Pass condition:

- active mirrored streams report `Sync ok`,
- no repeated `Sync warning` appears under normal runtime.

If you see warnings, it means the DT mirror exists but is drifting beyond the configured tolerance.

### Test 5: verify the thermal pipeline behind the DT topics

In a new terminal:

```bash
ros2 topic echo /thermal_reading --once
ros2 topic echo /thermal_map --once
ros2 topic hz /thermal_reading
ros2 topic hz /thermal_map
```

Then confirm the mirrored DT versions also respond:

```bash
ros2 topic echo /dt/thermal_reading --once
ros2 topic echo /dt/thermal_map --once
```

Pass condition:

- thermal topics publish on the ROS side,
- mirrored thermal topics publish on the DT side too.

### Fast smoke test

If you only need a short demo check, do just these three things:

1. Launch `sim_with_bridge.launch.py` and confirm `/clock` works.
2. Launch `dt_integration.launch.py` and confirm `/dt/odom` exists.
3. Publish once to `/dt/cmd_vel` and confirm the robot moves in Gazebo.

If all three pass, the whole bridge chain is basically proven end to end.

## Common confusion points

### "Is this using `rosbridge_suite`?"

No. This repo does not show a WebSocket JSON bridge like `rosbridge_suite`. The bridge here is `ros_gz_bridge`, which is a ROS 2 <-> Gazebo Sim message bridge.

### "Why not publish directly to `/cmd_vel` from the DT side?"

Because this stack expects `/cmd_vel` as `geometry_msgs/msg/TwistStamped`, while the DT interface intentionally stays simpler as `geometry_msgs/msg/Twist`. `dt_mediator` handles that adaptation.

### "Why mirror topics into `/dt/*` if the originals already exist?"

The `/dt/*` namespace creates a clean, DT-facing contract. It isolates the digital twin interface from the project internals and makes synchronization checks easier.

## Practical mental model

The easiest way to think about the system is:

- `ros_gz_bridge` connects Gazebo to ROS 2.
- `dt_mediator` connects ROS 2 to the digital twin interface.
- `sync_monitor` tells you whether that DT interface is staying current.

So the full chain is not one bridge but two adapters in series:

Gazebo Sim
-> `ros_gz_bridge`
-> ROS 2 project nodes
-> `dt_mediator`
-> `/dt/*` digital twin interface

And for commands:

DT controller
-> `/dt/cmd_vel`
-> `dt_mediator`
-> `/cmd_vel`
-> `ros_gz_bridge`
-> Gazebo robot motion
