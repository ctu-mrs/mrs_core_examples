#!/usr/bin/env python3

import launch
import os
# import sys

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        EnvironmentVariable,
        )


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription()

    pkg_name = "sweeping_generator"

    this_pkg_path = get_package_share_directory(pkg_name)

    config = os.path.join(
        get_package_share_directory('sweeping_generator'),
        'config',
        'sweeping_generator.yaml'
        )

    # #{ uav_name 
    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=EnvironmentVariable('UAV_NAME',default_value='uav1'),
        description="The uav name used for namespacing",
    ))
    # # }

    # uav_name = LaunchConfiguration('uav_name')
    uav_name=os.getenv('UAV_NAME', "uav1")
    # # } end of uav_name.

    # # { sweeping_generator
    sweeping_generator = Node(

            package=pkg_name,
        
            namespace=uav_name,
            name='sweeeping_generator',
            executable='sweeping_generator',
            parameters=[config],

            remappings=[
                # # subscribers
                ("~/control_manager_diag_in","control_manager/diagnostics"),
                # service servers
                ("~/path_out","trajectory_generation/path"),
                # service clients
                ("~/start_in","start"),
            ],
        )

    ld.add_action(sweeping_generator)

    return ld