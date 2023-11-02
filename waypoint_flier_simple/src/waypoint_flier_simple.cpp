/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/* custom msgs of MRS group */
#include <mrs_msgs/ReferenceStamped.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>

#include <random>
//}

namespace waypoint_flier_simple
{

/* class WaypointFlierSimple //{ */

class WaypointFlierSimple : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  // | -------------------------- flags ------------------------- |

  /* is set to true when the nodelet is initialized, useful for rejecting callbacks that are called before the node is initialized */
  std::atomic<bool> is_initialized_ = false;

  /* by default, the nodelet is deactivated, it only starts publishing goals when activated */
  std::atomic<bool> active_ = false;

  /* by default, the nodelet is deactivated, it only starts publishing goals when activated */
  std::atomic<bool> have_odom_ = false;

  /* variables to store the coordinates of the current goal */
  double goal_x_ = 0.0;
  double goal_y_ = 0.0;
  double goal_z_ = 2.0;

  /* variables to store the maximum limit for the random waypoint generator */
  double max_x_;
  double max_y_;
  double max_z_;

  /* ROS messages which store the current reference and odometry */
  mrs_msgs::ReferenceStamped ref_;
  nav_msgs::Odometry         current_odom_;

  // | ---------------------- ROS subscribers --------------------- |
  ros::Subscriber sub_odom_;
  void            callbackOdom(const nav_msgs::Odometry& msg);

  // | ---------------------- ROS publishers --------------------- |
  ros::Publisher pub_reference_;

  // | ---------------------- ROS timers --------------------- |

  ros::Timer main_timer_;
  void       callbackMainTimer([[maybe_unused]] const ros::TimerEvent& te);

  // | ---------------------- ROS service servers --------------------- |

  bool               callbackStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_start_;

  // | ------------------ Additional functions ------------------ |

  double distance(const mrs_msgs::ReferenceStamped& waypoint, const nav_msgs::Odometry& odom);

  double getRandomDouble(double min, double max);
};
//}

/* onInit() //{ */

void WaypointFlierSimple::onInit() {

  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* initialize random number generator */
  srand(time(NULL));

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  nh.param("max_x", max_x_, 10.0);
  nh.param("max_y", max_y_, 10.0);
  nh.param("max_z", max_z_, 5.0);

  // | -------- initialize a subscriber for UAV odometry -------- |
  sub_odom_ = nh.subscribe("odom_in", 10, &WaypointFlierSimple::callbackOdom, this, ros::TransportHints().tcpNoDelay());

  // | -------- initialize a publisher for UAV reference -------- |
  pub_reference_ = nh.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);

  // | -- initialize the main timer - main loop of the nodelet -- |
  main_timer_ = nh.createTimer(ros::Rate(10), &WaypointFlierSimple::callbackMainTimer, this);

  // | ---------------- initialize service server --------------- |
  srv_server_start_ = nh.advertiseService("start", &WaypointFlierSimple::callbackStart, this);
  is_initialized_   = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackControlManagerDiag() //{ */


void WaypointFlierSimple::callbackOdom(const nav_msgs::Odometry& msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_) {
    return;
  }
  // | -------------- save the current UAV odometry ------------- |
  current_odom_ = msg;
  have_odom_    = true;
}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackMainTimer() //{ */

void WaypointFlierSimple::callbackMainTimer([[maybe_unused]] const ros::TimerEvent& te) {

  if (!active_) {

    ROS_INFO_THROTTLE(1.0, "[WaypointFlierSimple]: Waiting  for activation");

  } else {

    // calculate the distance to the current reference
    double dist_to_ref = distance(ref_, current_odom_);
    ROS_INFO_STREAM_THROTTLE(1.0, "[WaypointFlierSimple]: Distance to reference: " << dist_to_ref);

    // if the distance is less than 1 meter, generate a new reference
    if (dist_to_ref < 1.0) {

      ROS_INFO_STREAM("[WaypointFlierSimple]: Goal reached!");

      // generate new goal
      goal_x_ = getRandomDouble(-max_x_, max_x_);
      goal_y_ = getRandomDouble(-max_y_, max_y_);
      goal_z_ = getRandomDouble(2.0, max_z_);

      ROS_INFO_STREAM("[WaypointFlierSimple]: New goal X: " << goal_x_ << " Y: " << goal_y_ << " Z: " << goal_z_);
    }

    // fill out the ROS message and publish it
    ref_.reference.position.x = goal_x_;
    ref_.reference.position.y = goal_y_;
    ref_.reference.position.z = goal_z_;
    ref_.reference.heading    = 0.0;
    pub_reference_.publish(ref_);
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackStart() */

bool WaypointFlierSimple::callbackStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  // | ------------------- activation service ------------------- |
  // only activates the main loop when the nodelet is initialized and receiving odometry

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[WaypointFlierSimple]: Cannot start waypoint following, nodelet is not initialized.");
    return true;
  }

  if (!have_odom_) {

    res.success = false;
    res.message = "Waypoint flier is not receiving odometry!";
    ROS_WARN("[WaypointFlierSimple]: Cannot start, nodelet is not receiving odometry!");
    return true;
  }

  active_ = true;

  ROS_INFO("[WaypointFlierSimple]: Starting waypoint following.");
  ROS_INFO_STREAM("[WaypointFlierSimple]: Goal X: " << goal_x_ << " Y: " << goal_y_ << " Z: " << goal_z_);

  res.success = true;
  res.message = "Starting waypoint following.";

  return true;
}

//}

// | -------------------- support functions ------------------- |

/* distance() //{ */

double WaypointFlierSimple::distance(const mrs_msgs::ReferenceStamped& waypoint, const nav_msgs::Odometry& odom) {

  // | ------------- distance between two 3D points ------------- |

  return sqrt((pow(waypoint.reference.position.x - odom.pose.pose.position.x, 2)) + (pow(waypoint.reference.position.y - odom.pose.pose.position.y, 2)) +
              (pow(waypoint.reference.position.z - odom.pose.pose.position.z, 2)));
}

double WaypointFlierSimple::getRandomDouble(double min, double max) {

  // | --------- random double in the min and max bounds -------- |

  float r = (float)rand() / (float)RAND_MAX;
  return min + r * (max - min);
}

//}

}  // namespace waypoint_flier_simple

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(waypoint_flier_simple::WaypointFlierSimple, nodelet::Nodelet);
