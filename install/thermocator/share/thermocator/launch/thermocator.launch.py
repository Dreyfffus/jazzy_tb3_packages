#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )

    hot_threshold_arg = DeclareLaunchArgument(
        "hot_threshold",
        default_value="60.0",
        description="Temperature above which a cell is considered hot",
    )

    cold_threshold_arg = DeclareLaunchArgument(
        "cold_threshold",
        default_value="25.0",
        description="Temperature below which a cell is considered safe",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="1.0",
        description="Thermal map publish rate in Hz",
    )

    map_frame_arg = DeclareLaunchArgument(
        "map_frame", default_value="map", description="TF frame for the map"
    )

    robot_frame_arg = DeclareLaunchArgument(
        "robot_frame",
        default_value="base_footprint",
        description="TF frame for the robot base",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    hot_threshold = LaunchConfiguration("hot_threshold")
    cold_threshold = LaunchConfiguration("cold_threshold")
    publish_rate = LaunchConfiguration("publish_rate")
    map_frame = LaunchConfiguration("map_frame")
    robot_frame = LaunchConfiguration("robot_frame")

    # Starts immediately — needs to be publishing before the map builder
    # starts processing readings
    thermal_broadcaster_node = Node(
        package="thermocator",
        executable="thermal_broadcaster",
        name="thermal_broadcaster",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": map_frame,
                "robot_frame": robot_frame,
                "publish_rate": 5.0,
                "noise_stddev": 0.5,
                # Default heat zones — two zones in the Gazebo world
                # Override from command line if needed
                "zone_centers_x": [2.0, -1.0],
                "zone_centers_y": [2.0, 1.0],
                "zone_peak_temps": [80.0, 60.0],
                "zone_sigmas": [0.5, 0.3],
            }
        ],
    )

    # Delayed slightly to give the TF tree and /map time to be available
    # OneShotListener waits for /map internally but the TF tree needs
    # a moment after Nav2/SLAM starts
    thermal_map_builder_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="thermocator",
                executable="thermocator",
                name="thermal_map_builder",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": map_frame,
                        "robot_frame": robot_frame,
                        "hot_threshold": hot_threshold,
                        "cold_threshold": cold_threshold,
                        "publish_rate": publish_rate,
                        "ema_alpha": 0.2,
                        "gaussian_sigma": 0.15,
                        "min_confidence": 0.5,
                        "tf_timeout": 0.1,
                    }
                ],
            )
        ],
    )

    # Delayed longer — needs thermal map to be initialized and Nav2 to be
    # fully up before it starts sending goals
    decision_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="thermocator",
                executable="decision_node",
                name="decision_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": map_frame,
                        "robot_frame": robot_frame,
                        "hot_threshold": hot_threshold,
                        "frontier_min_distance": 0.5,
                        "investigation_duration": 3.0,
                        "control_rate": 1.0,
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            # Arguments
            use_sim_time_arg,
            hot_threshold_arg,
            cold_threshold_arg,
            publish_rate_arg,
            map_frame_arg,
            robot_frame_arg,
            # Nodes
            thermal_broadcaster_node,
            thermal_map_builder_node,
            decision_node,
        ]
    )
