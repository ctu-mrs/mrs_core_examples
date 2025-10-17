#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        LaunchConfiguration,
        EnvironmentVariable,
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "example_sweeping_generator"

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ sweeping generator node

    sweeping_generator = Node(
        package=pkg_name,
        namespace=uav_name,
        name='sweeping_generator',
        executable='sweeping_generator.py',
        parameters=[
            get_package_share_directory('example_sweeping_generator') + 'config/sweeping_generator.yaml',
        ],
        remappings=[
            # subscribers
            ("~/control_manager_diag_in", "control_manager/diagnostics"),
            # service servers
            ("~/path_out", "trajectory_generation/path"),
            # service clients
            ("~/start_in", "~/start"),
        ],
    )

    # #} end of sweeping generator node

    ld.add_action(sweeping_generator)

    return ld
