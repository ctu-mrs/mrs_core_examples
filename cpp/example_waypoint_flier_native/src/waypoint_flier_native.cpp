/* includes //{ */

// each ros package must have these
#include <rclcpp/rclcpp.hpp>

// for storing information about the state of the uav (position, twist) + covariances
#include <nav_msgs/msg/odometry.hpp>

// custom msgs of MRS group
#include <mrs_msgs/msg/reference_stamped.hpp>

// for calling simple ros services
#include <std_srvs/srv/trigger.hpp>

// helper functions for loading parameters
#include <params.h>

//}

/* using //{ */

using namespace std::chrono_literals;

//}

namespace example_waypoint_flier_native
{

/* class WaypointFlierNative //{ */

class WaypointFlierNative {
public:
  WaypointFlierNative(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
    return node_->get_node_base_interface();
  }

private:
  void initialize();

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  /* is set to true when the nodelet is initialized, useful for rejecting callbacks that are called before the node is initialized */
  std::atomic<bool> is_initialized_ = false;

  /* by default, the node component is deactivated, it only starts publishing goals when activated */
  std::atomic<bool> active_ = false;

  /* by default, the nodel component is deactivated, it only starts publishing goals when activated */
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
  mrs_msgs::msg::ReferenceStamped ref_;
  nav_msgs::msg::Odometry         current_odom_;

  // | ---------------------- ROS subscribers --------------------- |
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  void                                                     callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // | ---------------------- ROS publishers --------------------- |

  void                                                          callbackMainTimer();
  rclcpp::TimerBase::SharedPtr                                  timer_publisher_reference_;
  rclcpp::Publisher<mrs_msgs::msg::ReferenceStamped>::SharedPtr publisher_reference_;

  // | ---------------------- ROS service servers --------------------- |

  bool callbackStart([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_server_start_;

  // | ------------------ Additional functions ------------------ |

  double distance(const mrs_msgs::msg::ReferenceStamped& waypoint, const nav_msgs::msg::Odometry& odom);

  double getRandomDouble(double min, double max);
};

//}

/* WaypointFlierNative() //{ */

WaypointFlierNative::WaypointFlierNative(const rclcpp::NodeOptions& options) : node_(std::make_shared<rclcpp::Node>("WaypointFlierNative", options)) {

  initialize();
}

//}

/* initialize() //{ */

void WaypointFlierNative::initialize() {

  clock_ = node_->get_clock();

  bool loaded_successfully = true;

  /*------------ load parameters --------- */
  loaded_successfully &= waypoint_flier_native::utils::load_param("max_x", max_x_, 10.0, *node_);
  loaded_successfully &= waypoint_flier_native::utils::load_param("max_y", max_y_, 10.0, *node_);
  loaded_successfully &= waypoint_flier_native::utils::load_param("max_z", max_z_, 5.0, *node_);

  if (!loaded_successfully) {
    RCLCPP_INFO_ONCE(node_->get_logger(), "failed to load non-optional parameters");
    rclcpp::shutdown();
    exit(1);
  }

  // | -------- initialize a publisher for UAV reference -------- |

  publisher_reference_       = node_->create_publisher<mrs_msgs::msg::ReferenceStamped>("~/reference_out", 1);
  timer_publisher_reference_ = node_->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&WaypointFlierNative::callbackMainTimer, this));

  // | --------- initialize a subscriber for UAV Odometry -----------|

  const std::function<void(const nav_msgs::msg::Odometry::SharedPtr)> odom_cbk = std::bind(&WaypointFlierNative::callbackOdometry, this, std::placeholders::_1);
  sub_odom_                                                                    = node_->create_subscription<nav_msgs::msg::Odometry>("~/odom_in", 10, odom_cbk);

  // |--------------- service server for starting waypoint following ---------------|

  srv_server_start_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/start_waypoint_flying_in", std::bind(&WaypointFlierNative::callbackStart, this, std::placeholders::_1, std::placeholders::_2));

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;
}

//}

// |------------------ msg_callbacks ------------------------|

/* callbackOdometry() //{ */

void WaypointFlierNative::callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  // | -------------- save the current UAV odometry ------------- |

  current_odom_ = *msg;
  have_odom_    = true;
}

//}

// |------------------ timer callbacks ---------------------------|

/* callbackMainTimer() //{ */

void WaypointFlierNative::callbackMainTimer() {

  if (!active_) {

    RCLCPP_INFO(node_->get_logger(), "waypoint flier is not activated yet");

  } else {

    const double curr_dist = distance(ref_, current_odom_);

    if (curr_dist < 1.0) {

      RCLCPP_INFO_STREAM(node_->get_logger(), "goal reached!");

      /* select new reference point */
      goal_x_ = getRandomDouble(-max_x_, max_x_);
      goal_y_ = getRandomDouble(-max_y_, max_y_);
      goal_z_ = getRandomDouble(2, max_z_);

      RCLCPP_INFO_STREAM(node_->get_logger(), "new goal X: " << goal_x_ << " Y: " << goal_y_ << " Z: " << goal_z_);
    }

    ref_.reference.position.x = goal_x_;
    ref_.reference.position.y = goal_y_;
    ref_.reference.position.z = goal_z_;
    ref_.reference.heading    = 0.0;

    publisher_reference_->publish(ref_);
  }
}

//}

/* callbackStart() //{ */

bool WaypointFlierNative::callbackStart([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                        const std::shared_ptr<std_srvs::srv::Trigger::Response>                 res) {

  // | ------------------- activation service ------------------- |
  // only activates the main loop when the nodelet is initialized and receiving odometry

  if (!is_initialized_) {

    res->success = false;
    res->message = "waypoint flier not initialized!";
    RCLCPP_WARN(node_->get_logger(), "cannot start waypoint following, nodelet is not initialized.");
    return true;
  }

  if (!have_odom_) {

    res->success = false;
    res->message = "waypoint flier is not receiving odometry!";
    RCLCPP_WARN(node_->get_logger(), "cannot start, nodelet is not receiving odometry!");
    return true;
  }

  active_ = true;

  RCLCPP_INFO(node_->get_logger(), "starting waypoint following");
  RCLCPP_INFO_STREAM(node_->get_logger(), "Goal X: " << goal_x_ << " Y: " << goal_y_ << " Z: " << goal_z_);

  res->success = true;
  res->message = "starting waypoint following";

  return true;
}

//}

// | -------------------- support functions ------------------- |

/* distance() //{ */

double WaypointFlierNative::distance(const mrs_msgs::msg::ReferenceStamped& waypoint, const nav_msgs::msg::Odometry& odom) {

  return sqrt((pow(waypoint.reference.position.x - odom.pose.pose.position.x, 2)) + (pow(waypoint.reference.position.y - odom.pose.pose.position.y, 2)) +
              (pow(waypoint.reference.position.z - odom.pose.pose.position.z, 2)));
}

//}

/* getRandomDouble() //{ */

double WaypointFlierNative::getRandomDouble(double min, double max) {

  // | --------- random double in the min and max bounds -------- |

  float r = (float)rand() / (float)RAND_MAX;
  return min + r * (max - min);
}

//}

}  // namespace example_waypoint_flier_native

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(example_waypoint_flier_native::WaypointFlierNative)
