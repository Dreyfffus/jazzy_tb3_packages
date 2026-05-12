#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -------------------------------------------------------------------------
    # Arguments
    # -------------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )

    map_frame_arg = DeclareLaunchArgument(
        "map_frame", default_value="map", description="TF frame for the map"
    )

    robot_frame_arg = DeclareLaunchArgument(
        "robot_frame",
        default_value="base_footprint",
        description="TF frame for the robot base",
    )

    # Thermal zone configuration
    zone_centers_x_arg = DeclareLaunchArgument(
        "zone_centers_x",
        default_value="[0.0, 1.5]",
        description="X coordinates of heat zone centers in map frame",
    )

    zone_centers_y_arg = DeclareLaunchArgument(
        "zone_centers_y",
        default_value="[0.0, -1.0]",
        description="Y coordinates of heat zone centers in map frame",
    )

    zone_peak_temps_arg = DeclareLaunchArgument(
        "zone_peak_temps",
        default_value="[80.0, 60.0]",
        description="Peak temperature of each heat zone in degrees C",
    )

    zone_sigmas_arg = DeclareLaunchArgument(
        "zone_sigmas",
        default_value="[1.2, 1.2]",
        description="Gaussian sigma (spread) of each heat zone in meters",
    )

    # Thermal map builder configuration
    cold_threshold_arg = DeclareLaunchArgument(
        "cold_threshold",
        default_value="0.0",
        description="Temperature mapped to occupancy value 0",
    )

    hot_threshold_arg = DeclareLaunchArgument(
        "hot_threshold",
        default_value="80.0",
        description="Temperature mapped to occupancy value 100",
    )

    min_confidence_arg = DeclareLaunchArgument(
        "min_confidence",
        default_value="0.5",
        description="Minimum confidence for a cell to be published",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="1.0",
        description="Thermal map publish rate in Hz",
    )

    # Decision node configuration
    heat_detection_threshold_arg = DeclareLaunchArgument(
        "heat_detection_threshold",
        default_value="20.0",
        description="Occupancy value (0-100) above which a cell is considered hot",
    )

    scoring_radius_arg = DeclareLaunchArgument(
        "scoring_radius",
        default_value="1.5",
        description="Radius in meters used to score frontier candidates",
    )

    w_unknown_hot_arg = DeclareLaunchArgument(
        "w_unknown_hot",
        default_value="3.0",
        description="Weight: unknown cells adjacent to hot cells",
    )

    w_dist_hottest_arg = DeclareLaunchArgument(
        "w_dist_hottest",
        default_value="0.5",
        description="Weight: penalty for distance from hottest known cell",
    )

    w_cold_penalty_arg = DeclareLaunchArgument(
        "w_cold_penalty",
        default_value="0.2",
        description="Weight: penalty for confirmed cold cells in scoring radius",
    )

    frontier_min_distance_arg = DeclareLaunchArgument(
        "frontier_min_distance",
        default_value="0.5",
        description="Minimum distance in meters for a frontier to be considered",
    )

    investigation_duration_arg = DeclareLaunchArgument(
        "investigation_duration",
        default_value="5.0",
        description="Seconds to wait at a goal before rescanning",
    )

    control_rate_arg = DeclareLaunchArgument(
        "control_rate",
        default_value="1.0",
        description="Decision node control loop rate in Hz",
    )

    # -------------------------------------------------------------------------
    # Substitutions
    # -------------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_frame = LaunchConfiguration("map_frame")
    robot_frame = LaunchConfiguration("robot_frame")
    zone_centers_x = LaunchConfiguration("zone_centers_x")
    zone_centers_y = LaunchConfiguration("zone_centers_y")
    zone_peak_temps = LaunchConfiguration("zone_peak_temps")
    zone_sigmas = LaunchConfiguration("zone_sigmas")
    cold_threshold = LaunchConfiguration("cold_threshold")
    hot_threshold = LaunchConfiguration("hot_threshold")
    min_confidence = LaunchConfiguration("min_confidence")
    publish_rate = LaunchConfiguration("publish_rate")
    heat_detection_threshold = LaunchConfiguration("heat_detection_threshold")
    scoring_radius = LaunchConfiguration("scoring_radius")
    w_unknown_hot = LaunchConfiguration("w_unknown_hot")
    w_dist_hottest = LaunchConfiguration("w_dist_hottest")
    w_cold_penalty = LaunchConfiguration("w_cold_penalty")
    frontier_min_distance = LaunchConfiguration("frontier_min_distance")
    investigation_duration = LaunchConfiguration("investigation_duration")
    control_rate = LaunchConfiguration("control_rate")

    # -------------------------------------------------------------------------
    # Nodes
    # -------------------------------------------------------------------------

    # Starts immediately -- must be publishing before map builder starts
    thermal_broadcaster = Node(
        package="thermocator",
        executable="thermal_broadcaster",
        name="thermal_broadcaster",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": map_frame,
                "robot_frame": robot_frame,
                "publish_rate": 2.5,
                "noise_stdev": 0.5,
                "zone_centers_x": zone_centers_x,
                "zone_centers_y": zone_centers_y,
                "zone_peak_temps": zone_peak_temps,
                "zone_sigmas": zone_sigmas,
            }
        ],
    )

    # Delayed 3s -- needs /map from slam_toolbox and TF tree to be stable
    thermal_map_builder = TimerAction(
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
                        "cold_threshold": cold_threshold,
                        "hot_threshold": hot_threshold,
                        "min_confidence": min_confidence,
                        "publish_rate": publish_rate,
                        "tf_timeout": 0.1,
                    }
                ],
            )
        ],
    )

    # Delayed 8s -- needs thermal map initialized and Nav2 fully active
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
                        "heat_detection_threshold": heat_detection_threshold,
                        "scoring_radius": scoring_radius,
                        "w_unknown_hot": w_unknown_hot,
                        "w_dist_hottest": w_dist_hottest,
                        "w_cold_penalty": w_cold_penalty,
                        "frontier_min_distance": frontier_min_distance,
                        "investigation_duration": investigation_duration,
                        "control_rate": control_rate,
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            # Arguments
            use_sim_time_arg,
            map_frame_arg,
            robot_frame_arg,
            zone_centers_x_arg,
            zone_centers_y_arg,
            zone_peak_temps_arg,
            zone_sigmas_arg,
            cold_threshold_arg,
            hot_threshold_arg,
            min_confidence_arg,
            publish_rate_arg,
            heat_detection_threshold_arg,
            scoring_radius_arg,
            w_unknown_hot_arg,
            w_dist_hottest_arg,
            w_cold_penalty_arg,
            frontier_min_distance_arg,
            investigation_duration_arg,
            control_rate_arg,
            # Nodes
            thermal_broadcaster,
            thermal_map_builder,
            decision_node,
        ]
    )
