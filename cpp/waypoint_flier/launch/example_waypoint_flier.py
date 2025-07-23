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

    pkg_name = "example_waypoint_flier"

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
    
    ld.add_action(ComposableNodeContainer(

        namespace=uav_name,
        name='waypoint_flier_container',
        
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        # arguments = ['--ros-args', '--log-level', LaunchConfiguration('log_level')],

        composable_node_descriptions=[

            ComposableNode(

                package=pkg_name,
                plugin='example_waypoint_flier::ExampleWaypointFlier',
                namespace=uav_name,
                name='example_waypoint_flier',

                parameters=[
                    {"uav_name": uav_name}, 
                    {"topic_prefix": "/" + uav_name},
                    {"enable_profiler": False},
                    {"config": this_pkg_path+'/config/example_waypoint_flier.yaml'},        
                    # {"use_sim_time": use_sim_time},

                ],

                remappings=[
                    # # subscribers
                    ("~/odom_in","estimation_manager/odom_main"),
                    ("~/control_manager_diagnostics_in","control_manager/diagnostics"),
                    ("~/odom_gt_in","ground_truth"),
                    # publishers
                    ("~/reference_out","control_manager/reference"),
                    ("~/dist_to_waypoint_out","dist_to_waypoint"),
                    # service servers
                    ("~/start_waypoints_following_in","~/start_waypoints_following"),
                    ("~/stop_waypoints_following_in","~/stop_waypoints_following"),
                    ("~/fly_to_first_waypoint_in","~/fly_to_first_waypoint"),
                    # service client
                    ("~/land_out","uav_manager/land"),
                ],
            )
        ],
    ))
    
    return ld

    


    

