#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import ComposableNodeContainer , LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        LaunchConfiguration,
        )

# Good source to understand translation between ros1 and ros2.
## https://github.com/MetroRobots/rosetta_launch?tab=readme-ov-file

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "example_waypoint_flier_native"

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ standalone

    ld.add_action(DeclareLaunchArgument(
       'standalone',
       default_value='true'
    ))

    standalone = LaunchConfiguration('standalone')

    # #} end of standalone

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # #{ waypoint flier native node

    node = ComposableNode(
        package=pkg_name,
        plugin='example_waypoint_flier_native::WaypointFlierNative',
        namespace=uav_name,
        name='waypoint_flier_native',
        parameters=[
            {"uav_name": uav_name},
            {"use_sim_time": use_sim_time},
            {"config": this_pkg_path+'/config/waypoint_flier_native.yaml'},
        ],
        remappings=[
            # # subscribers
            ("~/odom_in", "estimation_manager/odom_main"),
            # publishers
            ("~/reference_out", "control_manager/reference"),
            # service servers
            ("~/start_waypoint_flying_in", "~/start_waypoint_flying"),
        ],
    )

    # #} end of waypoint flier native node

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ load into container

    load_into_existing = LoadComposableNodes(
        target_container = container_name,
        composable_node_descriptions = [node],
        condition = UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of load into container

    # #{ standalone container

    ld.add_action(ComposableNodeContainer(
        namespace=uav_name,
        name='waypoint_flier_native_container',
        package = 'rclcpp_components',
        executable = 'component_container_mt',
        output = 'screen',
        arguments = ['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[node],
        condition = IfCondition(standalone)
    ))

    # #} end of standalone container

    return ld
