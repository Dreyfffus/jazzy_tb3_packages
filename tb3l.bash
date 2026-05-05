#!/bin/bash

service="${1:-}"
robot_ip="${2:-}"

SETUP="source /opt/ros/jazzy/setup.bash && \
       source ~/turtlebot3_ws/install/setup.bash && \
       export TURTLEBOT3_MODEL=burger"

ROBOT_SETUP="source /opt/ros/jazzy/setup.bash && \
             source ~/turtlebot3_ws/install/setup.bash && \
             export TURTLEBOT3_MODEL=burger"

if [ -z "$service" ]; then
    echo "Usage: $0 <service> [robot_ip]"
    echo "Local services  : teleop | rviz | nav | thermal | broadcaster | build"
    echo "Robot services  : robot_thermal | robot_broadcaster | robot_bringup"
    echo "  (robot services require robot_ip as 2nd argument)"
    exit 1
fi

case "$service" in

    teleop)
        bash -c "${SETUP} && \
                 ros2 run turtlebot3_teleop teleop_keyboard"
        ;;

    rviz)
        bash -c "${SETUP} && \
                 ros2 launch turtlebot3_cartographer cartographer.launch.py"
        ;;

    nav)
        bash -c "${SETUP} && \
                 ros2 launch turtlebot3_navigation2 navigation2.launch.py \
                 params_file:=~/turtlebot3_ws/src/thermocator/config/nav2_thermal_params.yaml"
        ;;

    thermal)
        bash -c "${SETUP} && \
                 ros2 run thermocator thermocator"
        ;;

    broadcaster)
        bash -c "${SETUP} && \
                 ros2 run thermocator thermal_broadcaster"
        ;;

    build)
        package="${2:-}"
        if [ -n "$package" ]; then
            echo "Building package: $package"
            bash -c "${SETUP} && \
                     cd ~/turtlebot3_ws && \
                     colcon build --packages-select ${package} \
                     --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
        else
            echo "Building all packages ..."
            bash -c "${SETUP} && \
                     cd ~/turtlebot3_ws && \
                     colcon build \
                     --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
        fi
        ;;

    launch)
        bash -c "${SETUP} && \
             ros2 launch thermocator thermocator.launch.py \
             use_sim_time:=false"
        ;;


    robot_thermal | robot_broadcaster | robot_bringup)
        if [ -z "$robot_ip" ]; then
            echo "Error: robot services require an IP address as 2nd argument"
            echo "Usage: $0 $service <robot_ip>"
            exit 1
        fi

        echo "Checking SSH connectivity to $robot_ip ..."
        if ! ssh -o ConnectTimeout=5 -o BatchMode=yes \
                 ubuntu@$robot_ip exit 2>/dev/null; then
            echo "Error: cannot reach robot at $robot_ip"
            echo "Make sure the robot is on and SSH keys are set up."
            exit 1
        fi
        echo "Connected to robot at $robot_ip"

        case "$service" in
            robot_thermal)
                echo "Starting ThermalMapBuilder on robot ..."
                ssh -t ubuntu@$robot_ip bash -c \
                    "'${ROBOT_SETUP} && \
                      ros2 run thermocator thermocator'"
                ;;

            robot_broadcaster)
                echo "Starting ThermalBroadcaster on robot ..."
                ssh -t ubuntu@$robot_ip bash -c \
                    "'${ROBOT_SETUP} && \
                      ros2 run thermocator thermal_broadcaster'"
                ;;

            robot_bringup)
                echo "Starting TurtleBot3 bringup on robot ..."
                ssh -t ubuntu@$robot_ip bash -c \
                    "'${ROBOT_SETUP} && \
                      ros2 launch turtlebot3_bringup robot.launch.py'"
                ;;
        esac
        ;;

    *)
        echo "Error: unknown service '$service'"
        echo "Local services  : teleop | rviz | nav | thermal | broadcaster | build"
        echo "Robot services  : robot_thermal | robot_broadcaster | robot_bringup"
        exit 1
        ;;
esac
