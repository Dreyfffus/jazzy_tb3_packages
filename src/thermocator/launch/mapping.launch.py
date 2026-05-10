import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_thermocator = get_package_share_directory("thermocator")
    pkg_slam_toolbox = get_package_share_directory("slam_toolbox")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    slam_params_file = os.path.join(
        pkg_thermocator, "config", "slam_toolbox_params.yaml"
    )

    nav2_params_file = os.path.join(pkg_thermocator, "config", "nav2_slam_params.yaml")

    thermocator_launch_file = os.path.join(
        pkg_thermocator, "launch", "thermocator.launch.py"
    )

    # -------------------------------------------------------------------------
    # slam_toolbox
    # First to launch. Owns /map and the map->odom transform.
    # Everything else depends on these being available.
    # -------------------------------------------------------------------------
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "params_file": slam_params_file,
            "use_sim_time": "true",
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Nav2
    # autostart:=false — disables nav2_bringup's own lifecycle manager so it
    # does not attempt to activate docking_server. Our lifecycle manager below
    # takes full control of node activation.
    # -------------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
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
    # map_server and amcl are excluded — slam_toolbox owns those roles.
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

    nav2_delayed = TimerAction(period=5.0, actions=[nav2_launch, lifecycle_manager])

    # -------------------------------------------------------------------------
    # Thermocator stack
    # Delayed 10 seconds — after slam_toolbox (0s) and Nav2 (5s) are both up.
    #   0s  -> slam_toolbox
    #   5s  -> Nav2 + lifecycle manager
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
            slam_toolbox_launch,
            nav2_delayed,
            thermocator_delayed,
        ]
    )
