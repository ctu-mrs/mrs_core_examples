#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np

from mrs_msgs.msg import ControlManagerDiagnostics,Reference
from mrs_msgs.srv import PathSrv,PathSrv_Request
from mrs_msgs.srv import Vec1,Vec1Response

class SweepingGenerator(Node):

    # #{ __init__(self)

    def __init__(self):
        super().__init__('sweeping_generator')
        self.frame_id = self.get_parameter("~frame_id").value

        self.center_x = self.get_parameter("~center/x").value
        self.center_y = self.get_parameter("~center/y").value
        self.center_z = self.get_parameter("~center/z").value

        self.dimensions_x = self.get_parameter("~dimensions/x").value
        self.dimensions_y = self.get_parameter("~dimensions/y").value

        self.timer_main_rate = self.get_parameter("~timer_main/rate").value

        rclpy.loginfo('[SweepingGenerator]: initialized')
        
        self.sub_control_manager_diag = rclpy.Subscriber("~control_manager_diag_in", ControlManagerDiagnostics, self.callbackControlManagerDiagnostics)

        ## | --------------------- service servers -------------------- |

        self.ss_start = rclpy.Service('~start_in', Vec1, self.callbackStart)

        ## | --------------------- service clients -------------------- |

        self.sc_path = rclpy.ServiceProxy('~path_out', PathSrv)

        ## | ------------------------- timers ------------------------- |

        self.timer_main = rclpy.Timer(rclpy.Duration(1.0/self.timer_main_rate), self.timerMain)

        ## | -------------------- spin till the end ------------------- |

        self.is_initialized = True

        rclpy.spin()

    # #} end of __init__()

    ## | ------------------------- methods ------------------------ |

    # #{ planPath()
    def plan_path(self, step_size):
        self.get_logger().info("[SweepingGenerator]: planning path")

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
                path_msg.path.points.append(point)

            sign *= -1.0

        return path_msg
    
    # #} end of planPath()

    ## | ------------------------ callbacks ----------------------- |

    # #{ callbackControlManagerDiagnostics():

    def callback_control_manager_diagnostics(self, msg):
        if not self.is_initialized:
            return
        self.get_logger().info_once("[SweepingGenerator]: getting ControlManager diagnostics")
        self.control_manager_diag = msg
    
    # #} end of

    # #{ callbackStart():
    def callback_start(self, request, response):
        if not self.is_initialized:
            response.success = False
            response.message = "not initialized"
            return response

        step_size = request.goal
        path_msg = self.plan_path(step_size)

        future = self.cli_path.call_async(path_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info("[SweepingGenerator]: path set")
            else:
                self.get_logger().warn(f"[SweepingGenerator]: path setting failed: {result.message}")
        else:
            self.get_logger().error("[SweepingGenerator]: path service call failed")

        response.success = True
        response.message = "starting"
        return response

    def timer_main_callback(self):
        if not self.is_initialized:
            return

        self.get_logger().info_once("[SweepingGenerator]: main timer spinning")

        if isinstance(self.control_manager_diag, ControlManagerDiagnostics):
            if self.control_manager_diag.tracker_status.have_goal:
                self.get_logger().info("[SweepingGenerator]: tracker has goal")
            else:
                self.get_logger().info("[SweepingGenerator]: waiting for command")


def main(args=None):
    rclpy.init(args=args)
    node = SweepingGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        node = Node()
    except rclpy.ROSInterruptException:
        pass

