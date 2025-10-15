#!/usr/bin/env python3

import launch

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        LaunchConfiguration
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "example_plugin_manager"

    this_pkg_path = get_package_share_directory(pkg_name)

    namespace = 'example_plugin_manager',

    # #{ standalone

    ld.add_action(DeclareLaunchArgument(
        'standalone',
        default_value='true'
    ))

    standalone = LaunchConfiguration('standalone')

    # #} end of standalone

    # #{ example plugin manager

    example_plugin_manager = ComposableNode(
        package=pkg_name,
        plugin='example_plugin_manager::ExamplePluginManager',
        namespace=namespace,
        name='example_plugin_manager',
        parameters=[
            {"config": this_pkg_path+'/config/example_plugin_manager.yaml'},
            {"plugin_config": this_pkg_path + '/config/plugins.yaml'},

        ],
    )

    # #} end of example plugin manager

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    load_into_existing = LoadComposableNodes(
        target_container = container_name,
        composable_node_descriptions = [example_plugin_manager],
        condition = UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # # } end of waypoint flier example

    # # { standalone container

    ld.add_action(ComposableNodeContainer(
        name= 'example_plugin_manager_container',
        namespace='example_plugin_manager',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        # arguments = ['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[example_plugin_manager],
        condition = IfCondition(standalone)
    ))

    # # }

    return ld
