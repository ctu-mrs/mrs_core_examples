#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from mrs_msgs.msg import ControlManagerDiagnostics,Reference
from mrs_msgs.srv import PathSrv,PathSrv_Request
from mrs_msgs.srv import Vec1,Vec1_Response

class SweepingGenerator(Node):

    def __init__(self):

        super().__init__('sweeping_generator')

        self.cbkgr_ss = MutuallyExclusiveCallbackGroup() # for service servers
        self.cbkgr_sc = MutuallyExclusiveCallbackGroup() # for service client
        self.cbkgr_timers = MutuallyExclusiveCallbackGroup() # for timers

        self.declare_parameter('frame_id', "world_origin")

        self.declare_parameter("center.x", 0.0)
        self.declare_parameter("center.y", 0.0)
        self.declare_parameter("center.z", 3.0)

        self.declare_parameter("dimensions.x", 10.0)
        self.declare_parameter("dimensions.y", 10.0)

        self.declare_parameter("timer_main.rate", 1.0)

        self.frame_id = self.get_parameter('frame_id').value

        self.center_x = self.get_parameter("center.x").value
        self.center_y = self.get_parameter("center.y").value
        self.center_z = self.get_parameter("center.z").value

        self.dimensions_x = self.get_parameter("dimensions.x").value
        self.dimensions_y = self.get_parameter("dimensions.y").value

        self.timer_main_rate = self.get_parameter("timer_main.rate").value
        
        self.sub_control_manager_diag = self.create_subscription(ControlManagerDiagnostics, "~/control_manager_diag_in", self.callback_control_manager_diagnostics, 10)

        ## | --------------------- service servers -------------------- |

        self.ss_start = self.create_service(Vec1, "~/start_in", self.callback_start, callback_group=self.cbkgr_ss)

        ## | --------------------- service clients -------------------- |

        self.sc_path = self.create_client(PathSrv,"~/path_out", callback_group=self.cbkgr_sc)
        while not self.sc_path.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("waiting for path_out service...")

        ## | ------------------------- timers ------------------------- |

        self.timer_main = self.create_timer(1.0/self.timer_main_rate, self.timer_main_callback, callback_group=self.cbkgr_timers)

        self.is_initialized = True

        self.get_logger().info('initialized')

    ## | ------------------------- methods ------------------------ |

    def plan_path(self, step_size):

        self.get_logger().info("planning path")

        path_msg = PathSrv_Request()

        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = self.get_clock().now().to_msg()

        path_msg.path.fly_now = True
        path_msg.path.use_heading = True

        sign = 1.0

        for i in np.arange(-self.dimensions_x/2.0, self.dimensions_x/2.0, step_size):

            for j in np.arange(-self.dimensions_y/2.0, self.dimensions_y/2.0, step_size):
                point = Reference()
                point.position.x = self.center_x + i
                point.position.y = self.center_y + j*sign
                point.position.z = self.center_z
                point.heading = 0.0
                self.get_logger().info(f"point_pos_x: {self.center_x + i}, point_pos_y: {self.center_y + j*sign} point_pos_z: {self.center_z}\n")
                path_msg.path.points.append(point)

            sign *= -1.0
            
        return path_msg

    ## | ------------------------ callbacks ----------------------- |

    def callback_control_manager_diagnostics(self, msg):

        if not self.is_initialized:
            return

        self.get_logger().info("getting ControlManager diagnostics", once=True)
        self.sub_control_manager_diag = msg

    def callback_start(self, request, response):

        if not self.is_initialized:
            response.success = False
            response.message = "not initialized"
            return response

        step_size = request.goal
        path_msg = self.plan_path(step_size)

        future = self.sc_path.call_async(path_msg)

        # self.get_logger().info("client for path generation requested the service.")
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info("path set")
            else:
                self.get_logger().warn(f"path setting failed: {result.message}")
        else:
            self.get_logger().error("path service call failed")

        response.success = True
        response.message = "starting"

        return response

    def timer_main_callback(self):

        if not self.is_initialized:
            return
        
        self.get_logger().info("main timer spinning", once=True)

        if isinstance(self.sub_control_manager_diag, ControlManagerDiagnostics):

            if self.sub_control_manager_diag.tracker_status.have_goal:
                self.get_logger().info("tracker has goal")
            else:
                self.get_logger().info("waiting for command")

def main(args=None):

    rclpy.init(args=args)

    node = SweepingGenerator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
