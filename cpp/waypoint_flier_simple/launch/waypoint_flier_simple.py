#!/usr/bin/env python3

import launch
import os
import sys

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
        LaunchConfiguration,        
        PathJoinSubstitution,
        EnvironmentVariable,
        )

## https://github.com/MetroRobots/rosetta_launch?tab=readme-ov-file

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "example_waypoint_flier_simple"

    this_pkg_path = get_package_share_directory(pkg_name)


    # #{ uav_name 
    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=EnvironmentVariable('UAV_NAME',default_value='uav1'),
        description="The uav name used for namespacing",
    ))
    
    # uav_name = LaunchConfiguration('uav_name')
    uav_name=os.getenv('UAV_NAME', "uav1")
    # #} end of custom_config

    namespace = uav_name,

    ld.add_action(DeclareLaunchArgument(
        'DEBUG',
        default_value='false',
        description='if ran using GNU debugger'
    ))

    ld.add_action(DeclareLaunchArgument(
       'standalone',
       default_value='false'
    ))

    ld.add_action(ComposableNodeContainer(

        namespace=uav_name,
        name='waypoint_flier_simple_container',
        
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        # arguments = ['--ros-args', '--log-level', LaunchConfiguration('log_level')],

        composable_node_descriptions=[

            ComposableNode(

                package=pkg_name,
                plugin='example_waypoint_flier_simple::WaypointFlierSimple',
                namespace=uav_name,
                name='example_waypoint_flier_simple',

                parameters=[
                    {"uav_name": uav_name}, 
                    {"topic_prefix": "/" + uav_name},
                    {"enable_profiler": False},
                    {"config": this_pkg_path+'/config/waypoint_flier_simple.yaml'},        

                ],

                remappings=[
                    # # subscribers
                    ("~/odom_in","estimation_manager/odom_main"),
                    # publishers
                    ("~/reference_out","control_manager/reference"),
                    # service service
                    ("~/start", "~/start"),
                ],
            )
        ],
    ))
    
    return ld