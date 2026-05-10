import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_thermocator = get_package_share_directory("thermocator")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    nav2_params_file = os.path.join(
        pkg_thermocator, "config", "nav2_thermal_params.yaml"
    )

    map_file = os.path.join(pkg_thermocator, "worlds", "my_map.yaml")

    thermocator_launch_file = os.path.join(
        pkg_thermocator, "launch", "thermocator.launch.py"
    )

    # -------------------------------------------------------------------------
    # Nav2 localization
    # Runs map_server and amcl via their own lifecycle manager inside
    # localization_launch.py. This is separate from the navigation lifecycle
    # manager below and does not include docking_server.
    # -------------------------------------------------------------------------
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": "true",
            "map": map_file,
            "autostart": "true",
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Nav2 navigation stack
    # autostart:=false — disables nav2_bringup's own lifecycle manager so it
    # does not attempt to activate docking_server. Our lifecycle manager below
    # takes full control of node activation.
    # -------------------------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
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
    # Our lifecycle manager
    # Replaces nav2_bringup's hardcoded list which includes docking_server.
    # map_server and amcl are excluded — they are managed by localization_launch.
    # route_server excluded — not relevant for this robot.
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

    # -------------------------------------------------------------------------
    # Thermocator stack
    # Delayed 10 seconds to give Nav2 time to fully activate before the
    # thermal map builder and decision node start requesting TF and map data.
    #   0s  -> Nav2 localization + navigation + lifecycle manager
    #   10s -> thermal_broadcaster
    #   13s -> thermal_map_builder
    #   18s -> decision_node
    # -------------------------------------------------------------------------
    thermocator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thermocator_launch_file),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    thermocator_delayed = TimerAction(period=10.0, actions=[thermocator_launch])

    return LaunchDescription(
        [
            localization_launch,
            navigation_launch,
            lifecycle_manager,
            thermocator_delayed,
        ]
    )
