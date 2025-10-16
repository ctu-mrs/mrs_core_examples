import time
import unittest
import os
import sys

import launch
import launch_ros
import launch_testing.actions
import launch_testing.asserts
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
import rclpy
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool

def generate_test_description():

    SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

    this_pkg="example_tracker_plugin"

    ld = launch.LaunchDescription()

    launch_file_path = os.path.abspath(__file__)
    launch_dir = os.path.dirname(launch_file_path)

    test_name = os.path.basename(launch_dir)

    ld.add_action(
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('example_plugin_manager'),
                            'launch',
                            'example_plugin_manager.launch.py'
                        ])
                    ])
                )
            ]
        )
    )

    # starts the python test part down below
    ld.add_action(
        launch.actions.TimerAction(
            period=1.0, actions=[launch_testing.actions.ReadyToTest()]),
        )

    return ld

class Test(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_logs_crashing(self, proc_output):
        proc_output.assertWaitFor('[ExamplePlugin]: initialized', timeout=5.0, stream='stderr')
        proc_output.assertWaitFor('[ExamplePlugin2]: initialized', timeout=5.0, stream='stderr')
        proc_output.assertWaitFor('somebody called', timeout=5.0, stream='stderr')

    def test_interactor(self, proc_output, timeout=10):
        time.sleep(timeout)

@launch_testing.post_shutdown_test()
class PublisherHandlerTestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
