#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_thermocator = get_package_share_directory("thermocator")
    pkg_cartographer = get_package_share_directory("cartographer_ros")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    nav2_params_file = os.path.join(pkg_thermocator, "config", "nav2_slam_params.yaml")

    thermocator_launch_file = os.path.join(
        pkg_thermocator, "launch", "thermocator.launch.py"
    )

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cartographer, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Nav2 — delayed 5s to give slam_toolbox time to publish /map and TF
    #
    # autostart:=false disables nav2_bringup's own lifecycle manager so it
    # does not attempt to activate nodes we have excluded (amcl, map_server,
    # docking_server). Our lifecycle manager below takes full control.
    # -------------------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": "true",
            "autostart": "false",
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Lifecycle manager — replaces nav2_bringup's hardcoded list
    #
    # Excluded intentionally:
    #   amcl        -- slam_toolbox owns localization
    #   map_server  -- slam_toolbox publishes /map
    #   route_server -- not used
    #   docking_server -- not used
    # -------------------------------------------------------------------------
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "velocity_smoother",
                    "collision_monitor",
                    "bt_navigator",
                    "waypoint_follower",
                ],
            }
        ],
    )

    nav2_delayed = TimerAction(period=5.0, actions=[nav2, lifecycle_manager])

    # -------------------------------------------------------------------------
    # Thermocator stack — delayed 10s
    #
    # Internal delays inside thermocator.launch.py add further staggering:
    #   0s  -> slam_toolbox
    #   5s  -> Nav2 + lifecycle manager
    #  10s  -> thermal_broadcaster        (immediate inside thermocator launch)
    #  13s  -> thermal_map_builder        (3s delay inside thermocator launch)
    #  18s  -> decision_node              (8s delay inside thermocator launch)
    # -------------------------------------------------------------------------
    thermocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thermocator_launch_file),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    thermocator_delayed = TimerAction(period=10.0, actions=[thermocator])

    return LaunchDescription(
        [
            cartographer,
            nav2_delayed,
            thermocator_delayed,
        ]
    )
