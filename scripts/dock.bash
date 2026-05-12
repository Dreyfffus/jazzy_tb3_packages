#!/bin/bash

attach="$1"
service="${2:-}"
package="${3:-}"

SETUP="source /opt/ros/jazzy/setup.bash && \
       source /opt/turtlebot3_ws/install/setup.bash && \
       source /ws/install/setup.bash && \
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
    echo "Usage: $0 <start | attach | remote> [service] [package]"
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
                     use_sim_time:=True \
                     params_file:=/ws/src/thermocator/config/slam_params.yaml"
                ;;

            nav_map)
                docker exec -it turtlebot3_container bash -c \
                    "${SETUP} && \
                     ros2 launch turtlebot3_navigation2 navigation2.launch.py \
                     use_sim_time:=True \
                     params_file:=/ws/src/thermocator/config/nav2_slam_params.yaml"
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

    *)
        echo "Error: first argument must be start | attach | remote"
        echo "Usage: $0 <start | attach | remote> [service] [package]"
        exit 1
        ;;
esac
