#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    tolerance_arg = DeclareLaunchArgument(
        "sync_tolerance_seconds",
        default_value="0.5",
        description="Maximum accepted DT stream delay before warnings",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    sync_tolerance = LaunchConfiguration("sync_tolerance_seconds")

    thermal_broadcaster = Node(
        package="thermocator",
        executable="thermal_broadcaster",
        name="thermal_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    thermocator = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="thermocator",
                executable="thermocator",
                name="thermal_map_builder",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    decision_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="thermocator",
                executable="decision_node",
                name="decision_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    dt_mediator = Node(
        package="thermocator",
        executable="dt_mediator",
        name="dt_mediator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    sync_monitor = Node(
        package="thermocator",
        executable="sync_monitor",
        name="sync_monitor",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "tolerance_seconds": sync_tolerance,
            }
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            tolerance_arg,
            thermal_broadcaster,
            thermocator,
            decision_node,
            dt_mediator,
            sync_monitor,
        ]
    )
