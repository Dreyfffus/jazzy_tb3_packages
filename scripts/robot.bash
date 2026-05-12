#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_ROOT=$(dirname "$SCRIPT_DIR")
CONFIG_DIR="${WS_ROOT}/src/thermocator/config"


service="${1:-}"
domain_id="${2:-30}"
package="${3:-}"

SETUP="source /opt/ros/jazzy/setup.bash && \
       source ${WS_ROOT}/install/setup.bash && \
       export TURTLEBOT3_MODEL=burger && \
       export ROS_DOMAIN_ID=${domain_id}"

if [ -z "$service" ]; then
    echo "Usage: $0 <service> [domain_id] [package]"
    echo ""
    echo "Services:"
    echo "  teleop       — keyboard teleoperation"
    echo "  rviz         — Cartographer SLAM + RViz2"
    echo "  nav          — Nav2 with thermal costmap params"
    echo "  thermal      — ThermalMapBuilder node"
    echo "  broadcaster  — ThermalBroadcaster (sim sensor)"
    echo "  decision     — DecisionNode (frontier exploration)"
    echo "  launch       — Full thermocator pipeline"
    echo "  map_save     — Save current map to config folder"
    echo "  build        — Build workspace (optional: [package])"
    echo ""
    echo "Resolved Paths:"
    echo "  Workspace Dir : ${WS_ROOT}"
    echo "  Config Dir    : ${CONFIG_DIR}"
    echo "domain_id defaults to 30 if not specified"
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
                 use_sim_time:=False \
                 params_file:=${CONFIG_DIR}/nav2_slam_params.yaml"
        ;;

    thermal)
        bash -c "${SETUP} && \
                 ros2 run thermocator thermocator"
        ;;

    broadcaster)
        bash -c "${SETUP} && \
                 ros2 run thermocator thermal_broadcaster"
        ;;

    decision)
        bash -c "${SETUP} && \
                 ros2 run thermocator decision_node"
        ;;

    launch)
        bash -c "${SETUP} && \
                 ros2 launch thermocator thermocator.launch.py \
                 use_sim_time:=false"
        ;;

    map_save)
        bash -c "${SETUP} && \
                 ros2 run nav2_map_server map_saver_cli \
                 -f ~/turtlebot3_ws/src/thermocator/config/my_map"
        ;;

    build)
        if [ -n "$package" ]; then
            echo "Building package: $package"
            bash -c "${SETUP} && \
                     cd ${WS_ROOT} && \
                     colcon build --packages-select ${package}"
        else
            echo "Building all packages ..."
            bash -c "${SETUP} && \
                     cd ${WS_ROOT} && \
                     colcon build"
        fi
        ;;

    *)
        echo "Error: unknown service '$service'"
        echo "Run $0 without arguments to see usage"
        exit 1
        ;;
esac
