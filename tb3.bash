#!/bin/bash

attach="$1"
service="${2:-}"
package="${3:-}"
robot_ip="${4:-}"

SETUP="source /opt/ros/jazzy/setup.bash && \
       source /opt/turtlebot3_ws/install/setup.bash && \
       source /ws/install/setup.bash && \
       export TURTLEBOT3_MODEL=burger"

ROBOT_SETUP="source /opt/ros/jazzy/setup.bash && \
             source ~/turtlebot3_ws/install/setup.bash && \
             export TURTLEBOT3_MODEL=burger"

# ── Helper — ensure container is running before exec ─────────────────────────
ensure_container() {
    if ! docker ps --format '{{.Names}}' | grep -q "^turtlebot3_container$"; then
        echo "Container not running — starting it in the background ..."
        docker run --rm -d --name turtlebot3_container --net=host \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /home/c2irr10/turtlebot3_ws:/ws \
            turtlebot3_ws bash -c \
            "${SETUP} && sleep infinity"
        # Give the container a moment to initialize
        sleep 1
        echo "Container started."
    else
        echo "Container already running."
    fi
}

if [ -z "$attach" ]; then
    echo "Usage: $0 <start | attach | remote | robot> [service] [package|ip]"
    exit 1
fi

case "$attach" in
    start)
        echo "Starting session ..."
        docker run --rm -it --name turtlebot3_container --net=host \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /home/c2irr10/turtlebot3_ws:/ws \
            turtlebot3_ws bash -c \
            "${SETUP} && exec bash"
        ;;

    attach)
        ensure_container
        echo "Attaching to session ..."
        docker exec -it turtlebot3_container bash -c \
            "${SETUP} && exec bash"
        ;;

    remote)
        ensure_container
        echo "Executing '$service' ..."
        case "$service" in
            teleop)
                docker exec -it turtlebot3_container bash -c \
                    "${SETUP} && \
                     ros2 run turtlebot3_teleop teleop_keyboard"
                ;;

            rviz)
                docker exec -it turtlebot3_container bash -c \
                    "${SETUP} && \
                     ros2 launch turtlebot3_cartographer cartographer.launch.py \
                     use_sim_time:=True"
                ;;

            sim)
                docker exec -it turtlebot3_container bash -c \
                    "${SETUP} && \
                     ros2 launch my_tb3_world new_world.launch.py"
                ;;

            slam)
                docker exec -it turtlebot3_container bash -c \
                    "${SETUP} && \
                     ros2 launch slam_toolbox online_async_launch.py \
                     use_sim_time:=True"
                ;;

            nav)
                docker exec -it turtlebot3_container bash -c \
                    "${SETUP} && \
                     ros2 launch turtlebot3_navigation2 navigation2.launch.py \
                     use_sim_time:=True \
                     params_file:=/ws/src/thermocator/config/nav2_thermal_params.yaml"
                ;;

            thermal)
                docker exec -it turtlebot3_container bash -c \
                    "${SETUP} && \
                     ros2 run thermocator thermocator \
                     --ros-args -p use_sim_time:=true"
                ;;

            broadcaster)
                docker exec -it turtlebot3_container bash -c \
                    "${SETUP} && \
                     ros2 run thermocator thermal_broadcaster \
                     --ros-args -p use_sim_time:=true"
                ;;

            decision)
                docker exec -it turtlebot3_container bash -c \
                    "${SETUP} && \
                     ros2 run thermocator decision_node \
                     --ros-args -p use_sim_time:=true"
                ;;

            build)
                if [ -n "$package" ]; then
                    echo "Building package: $package"
                    docker exec -it turtlebot3_container bash -c \
                        "${SETUP} && \
                         cd /ws && colcon build \
                         --packages-select ${package} \
                         --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
                else
                    echo "Building all packages ..."
                    docker exec -it turtlebot3_container bash -c \
                        "${SETUP} && \
                         cd /ws && colcon build \
                         --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
                fi
                ;;

            launch)
                    docker exec -it turtlebot3_container bash -c \
                        "${SETUP} && \
                         ros2 launch thermocator thermocator.launch.py \
                         use_sim_time:=true"
                ;;

            *)
                echo "Error: unknown service '$service'"
                echo "Available services:"
                echo "  teleop | rviz | sim | slam | nav | thermal | broadcaster | build"
                exit 1
                ;;
        esac
        ;;

    robot)
        # ── Validate IP ───────────────────────────────────────────────────
        if [ -z "$service" ]; then
            echo "Error: robot mode requires a service argument"
            echo "Usage: $0 robot <service> <ip>"
            echo "Available services: thermal | broadcaster | teleop | bringup"
            exit 1
        fi

        if [ -z "$robot_ip" ]; then
            echo "Error: robot mode requires an IP address as 4th argument"
            echo "Usage: $0 robot <service> <ip>"
            exit 1
        fi

        # ── Verify SSH connectivity before attempting anything ────────────
        echo "Checking SSH connectivity to $robot_ip ..."
        if ! ssh -o ConnectTimeout=5 -o BatchMode=yes \
                 ubuntu@$robot_ip exit 2>/dev/null; then
            echo "Error: cannot reach robot at $robot_ip"
            echo "Make sure the robot is on and SSH keys are set up."
            exit 1
        fi
        echo "Connected to robot at $robot_ip"

        case "$service" in
            thermal)
                echo "Starting ThermalMapBuilder on robot ..."
                ssh -t ubuntu@$robot_ip bash -c \
                    "'${ROBOT_SETUP} && \
                      ros2 run thermocator thermocator'"
                ;;

            broadcaster)
                echo "Starting ThermalBroadcaster on robot ..."
                ssh -t ubuntu@$robot_ip bash -c \
                    "'${ROBOT_SETUP} && \
                      ros2 run thermocator thermal_broadcaster'"
                ;;

            teleop)
                echo "Starting Teleop on robot ..."
                ssh -t ubuntu@$robot_ip bash -c \
                    "'${ROBOT_SETUP} && \
                      ros2 run turtlebot3_teleop teleop_keyboard'"
                ;;

            bringup)
                echo "Starting TurtleBot3 bringup on robot ..."
                ssh -t ubuntu@$robot_ip bash -c \
                    "'${ROBOT_SETUP} && \
                      ros2 launch turtlebot3_bringup robot.launch.py'"
                ;;

            *)
                echo "Error: unknown robot service '$service'"
                echo "Available robot services: thermal | broadcaster | teleop | bringup"
                exit 1
                ;;
        esac
        ;;

    *)
        echo "Error: first argument must be start | attach | remote"
        echo "Usage: $0 <start | attach | remote | robot> [service] [package|ip]"
        exit 1
        ;;
esac
