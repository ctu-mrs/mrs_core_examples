#!/usr/bin/python3

import rospy
import numpy

from mrs_msgs.msg import ControlManagerDiagnostics,Reference
from mrs_msgs.srv import PathSrv,PathSrvRequest
from mrs_msgs.srv import Vec1,Vec1Response

class Node:

    # #{ __init__(self)

    def __init__(self):

        rospy.init_node("sweeping_generator", anonymous=True)

        ## | --------------------- load parameters -------------------- |

        self.frame_id = rospy.get_param("~frame_id")

        self.center_x = rospy.get_param("~center/x")
        self.center_y = rospy.get_param("~center/y")
        self.center_z = rospy.get_param("~center/z")

        self.dimensions_x = rospy.get_param("~dimensions/x")
        self.dimensions_y = rospy.get_param("~dimensions/y")

        self.timer_main_rate = rospy.get_param("~timer_main/rate")

        rospy.loginfo('[SweepingGenerator]: initialized')

        ## | ----------------------- subscribers ---------------------- |

        self.sub_control_manager_diag = rospy.Subscriber("~control_manager_diag_in", ControlManagerDiagnostics, self.callbackControlManagerDiagnostics)

        ## | --------------------- service servers -------------------- |

        self.ss_start = rospy.Service('~start_in', Vec1, self.callbackStart)

        ## | --------------------- service clients -------------------- |

        self.sc_path = rospy.ServiceProxy('~path_out', PathSrv)

        ## | ------------------------- timers ------------------------- |

        self.timer_main = rospy.Timer(rospy.Duration(1.0/self.timer_main_rate), self.timerMain)

        ## | -------------------- spin till the end ------------------- |

        self.is_initialized = True

        rospy.spin()

    # #} end of __init__()

    ## | ------------------------- methods ------------------------ |

    # #{ planPath()

    def planPath(self, step_size):

        rospy.loginfo('[SweepingGenerator]: planning path')

        # https://ctu-mrs.github.io/mrs_msgs/srv/PathSrv.html
        # -> https://ctu-mrs.github.io/mrs_msgs/msg/Path.html
        path_msg = PathSrvRequest()

        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()

        path_msg.path.fly_now = True

        path_msg.path.use_heading = True

        sign = 1.0

        # fill in the path with a sweeping pattern
        for i in numpy.arange(-self.dimensions_x/2.0, self.dimensions_x/2.0, step_size):

            for j in numpy.arange(-self.dimensions_y/2.0, self.dimensions_y/2.0, step_size):

                # https://ctu-mrs.github.io/mrs_msgs/msg/Reference.html
                point = Reference()

                point.position.x = self.center_x + i
                point.position.y = self.center_y + j*sign
                point.position.z = self.center_z
                point.heading = 0.0

                path_msg.path.points.append(point)

            if sign > 0.0:
                sign = -1.0
            else:
                sign = 1.0

        return path_msg

    # #} end of planPath()

    ## | ------------------------ callbacks ----------------------- |

    # #{ callbackControlManagerDiagnostics():

    def callbackControlManagerDiagnostics(self, msg):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: getting ControlManager diagnostics')

        self.sub_control_manager_diag = msg

    # #} end of

    # #{ callbackStart():

    def callbackStart(self, req):

        if not self.is_initialized:
            return Vec1Response(False, "not initialized")

        # set the step size based on the service data
        step_size = req.goal

        path_msg = self.planPath(step_size)

        try:
            response = self.sc_path.call(path_msg)
        except:
            rospy.logerr('[SweepingGenerator]: path service not callable')
            pass

        if response.success:
            rospy.loginfo('[SweepingGenerator]: path set')
        else:
            rospy.loginfo('[SweepingGenerator]: path setting failed, message: {}'.format(response.message))

        return Vec1Response(True, "starting")

    # #} end of

    ## | ------------------------- timers ------------------------- |

    # #{ timerMain()

    def timerMain(self, event=None):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: main timer spinning')

        if isinstance(self.sub_control_manager_diag, ControlManagerDiagnostics):
            if self.sub_control_manager_diag.tracker_status.have_goal:
                rospy.loginfo('[SweepingGenerator]: tracker has goal')
            else:
                rospy.loginfo('[SweepingGenerator]: waiting for command')

    # #} end of timerMain()

if __name__ == '__main__':
    try:
        node = Node()
    except rospy.ROSInterruptException:
        pass
