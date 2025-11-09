/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <string.h>

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous manipulation by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

/* for storing information about the state of the uav( position, twist) + covariances*/
#include <nav_msgs/msg/odometry.hpp>

/* custom msgs of MRS group */
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/msg/reference_stamped.hpp>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/node.h>
#include <mrs_lib/service_server_handler.h>

/* for calling simple ros services */
#include <std_srvs/srv/trigger.hpp>

/* for operations with matrices */
#include <Eigen/Dense>

//}

/* defines //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

//}

namespace example_waypoint_flier
{

/* DynParams_t //{ */

struct DynParams_t
{
  double waypoint_idle_time;
  double rate_publish_dist;
};

//}

/* class WaypointFlier //{ */

class WaypointFlier : public mrs_lib::Node {
public:
  WaypointFlier(rclcpp::NodeOptions options);

  void initialize();

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  std::string _uav_name_;
  bool        is_initialized_ = false;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>                  sh_odometry_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;

  void              callbackControlManagerDiag(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr msg);
  std::atomic<bool> have_goal_        = false;
  std::atomic<bool> waypoint_reached_ = false;

  // | --------------------- timer callbacks -------------------- |

  void                                                     timerPublishDistToWaypoint();
  mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped> pub_dist_to_waypoint_;
  std::shared_ptr<TimerType>                               timer_publish_dist_to_waypoint_;

  void                                                       timerPublishSetReference();
  mrs_lib::PublisherHandler<mrs_msgs::msg::ReferenceStamped> pub_reference_;
  std::shared_ptr<TimerType>                                 timer_publisher_reference_;
  double                                                     _rate_timer_publisher_reference_;

  void                       timerCheckSubscribers();
  std::shared_ptr<TimerType> timer_check_subscribers_;
  double                     _rate_timer_check_subscribers_;

  // | ----------------- sevice server callbacks ---------------- |

  bool callbackStartWaypointFollowing(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> srv_server_start_waypoints_following_;

  bool callbackStopWaypointFollowing(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> srv_server_stop_waypoints_following_;

  bool callbackFlyToFirstWaypoint(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> srv_server_fly_to_first_waypoint_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> srv_client_land_;
  bool                                                  _land_end_;

  // | -------------------- loading waypoints ---------------------- |

  std::vector<mrs_msgs::msg::Reference> waypoints_;
  std::string                           _waypoints_frame_;
  bool                                  waypoints_loaded_ = false;
  mrs_msgs::msg::Reference              current_waypoint_;
  std::mutex                            mutex_current_waypoint_;
  int                                   idx_current_waypoint_;
  int                                   n_waypoints_;
  int                                   _n_loops_;
  int                                   c_loop_;
  std::mutex                            mutex_waypoint_idle_time_;
  Eigen::MatrixXd                       _offset_;

  // | ------------------- dynamic reconfigure ------------------ |

  std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;
  std::mutex                            mutex_drs_params_;
  DynParams_t                           drs_params_;

  void callbackRatePublishDist(const double param_value);

  // | --------------------- waypoint idling -------------------- |

  bool                       is_idling_ = false;
  std::shared_ptr<TimerType> timer_idling_;
  double                     _waypoint_desired_dist_;
  void                       timerIdling();

  // | -------------------- support functions ------------------- |

  std::vector<mrs_msgs::msg::Reference> matrixToPoints(const Eigen::MatrixXd& matrix);

  void offsetPoints(std::vector<mrs_msgs::msg::Reference>& points, const Eigen::MatrixXd& offset);

  double distance(const mrs_msgs::msg::Reference& waypoint, const geometry_msgs::msg::Pose& pose);
};

//}

/* WaypointFlier() //{ */

WaypointFlier::WaypointFlier(rclcpp::NodeOptions options) : Node("example_waypoint_flier", options) {
  initialize();
}

//}

/* initialize() //{ */

void WaypointFlier::initialize() {

  node_  = this->this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  mrs_lib::ParamLoader param_loader(node_);

  dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_drs_params_);

  param_loader.addYamlFileFromParam("config");

  // Dynparam_mgr has its own param loader.
  // This will replicate the params from our main param loader into the dynparam_mgr, so
  // that dynparam_
  dynparam_mgr_->get_param_provider().copyYamls(param_loader.getParamProvider());

  dynparam_mgr_->register_param("waypoint_idle_time", &drs_params_.waypoint_idle_time, mrs_lib::DynparamMgr::range_t<double>(0.0, 5.0));
  dynparam_mgr_->register_param("rate/publish_dist_to_waypoint", &drs_params_.rate_publish_dist, mrs_lib::DynparamMgr::range_t<double>(1.0, 100.0),
                                (std::function<void(const double&)>)std::bind(&WaypointFlier::callbackRatePublishDist, this, std::placeholders::_1));

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("n_loops", _n_loops_);
  param_loader.loadParam("waypoint_desired_distance", _waypoint_desired_dist_);
  param_loader.loadParam("waypoints_frame", _waypoints_frame_);
  param_loader.loadParam("rate/publish_reference", _rate_timer_publisher_reference_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.loadParam("land_at_the_end", _land_end_);

  /* load waypoints as a half-dynamic matrix from config file */
  Eigen::MatrixXd waypoint_matrix;

  param_loader.loadMatrixDynamic("waypoints", waypoint_matrix, -1, 4);  // -1 indicates the dynamic dimension

  waypoints_            = matrixToPoints(waypoint_matrix);
  n_waypoints_          = waypoints_.size();
  waypoints_loaded_     = true;
  idx_current_waypoint_ = 0;
  c_loop_               = 0;

  RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "" << n_waypoints_ << " waypoints loaded");
  RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "" << _n_loops_ << " loops requested");

  /* load offsets of all waypoints as statics matrix from the config file and adjust waypoints accordingly.*/
  param_loader.loadMatrixKnown("offset", _offset_, 1, 4);

  if (!param_loader.loadedSuccessfully() || !dynparam_mgr_->loaded_successfully()) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load non-optional parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.node_name                           = "WaypointFlier";
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_odometry_             = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/odom_in");
  sh_control_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diagnostics_in",
                                                                                                  &WaypointFlier::callbackControlManagerDiag, this);
  // | ----------------------- publishers ----------------------- |

  pub_dist_to_waypoint_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(node_, "~/dist_to_waypoint_out");
  pub_reference_        = mrs_lib::PublisherHandler<mrs_msgs::msg::ReferenceStamped>(node_, "~/reference_out");

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions opts_autostart;

  opts_autostart.node           = node_;
  opts_autostart.autostart      = true;
  opts_autostart.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fn = std::bind(&WaypointFlier::timerPublishDistToWaypoint, this);
    timer_publish_dist_to_waypoint_   = std::make_shared<TimerType>(opts_autostart, rclcpp::Rate(drs_params_.rate_publish_dist, clock_), callback_fn);
  }

  {
    std::function<void()> callback_fn = std::bind(&WaypointFlier::timerCheckSubscribers, this);
    timer_check_subscribers_          = std::make_shared<TimerType>(opts_autostart, rclcpp::Rate(_rate_timer_check_subscribers_, clock_), callback_fn);
  }

  mrs_lib::TimerHandlerOptions opts_no_autostart;

  opts_no_autostart.node           = node_;
  opts_no_autostart.autostart      = false;
  opts_no_autostart.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fn = std::bind(&WaypointFlier::timerPublishSetReference, this);
    timer_publisher_reference_        = std::make_shared<TimerType>(opts_no_autostart, rclcpp::Rate(_rate_timer_publisher_reference_, clock_), callback_fn);
  }

  // | --------------------- service servers -------------------- |

  srv_server_start_waypoints_following_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/start_waypoints_following_in", std::bind(&WaypointFlier::callbackStartWaypointFollowing, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  srv_server_stop_waypoints_following_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/stop_waypoints_following_in", std::bind(&WaypointFlier::callbackStopWaypointFollowing, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  srv_server_fly_to_first_waypoint_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/fly_to_first_waypoint_in", std::bind(&WaypointFlier::callbackFlyToFirstWaypoint, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  // | --------------------- service clients -------------------- |

  srv_client_land_ = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/land_out", cbkgrp_sc_);

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO_ONCE(node_->get_logger(), "initialized");

  is_initialized_ = true;
}

//}

/* callbackControlManagerDiag() //{ */

void WaypointFlier::callbackControlManagerDiag(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr diagnostics) {

  /* do not continue if the component is not initialized */
  if (!is_initialized_) {
    return;
  }

  // this routine can not work without the odometry
  if (!sh_odometry_.hasMsg()) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "received first control manager diagnostics msg");

  // get the variable under the mutex
  mrs_msgs::msg::Reference current_waypoint = mrs_lib::get_mutexed(mutex_current_waypoint_, current_waypoint_);

  // extract the pose part of the odometry
  geometry_msgs::msg::Pose current_pose = mrs_lib::getPose(sh_odometry_.getMsg());

  double dist = distance(current_waypoint, current_pose);

  if (have_goal_ && !diagnostics->tracker_status.have_goal) {

    have_goal_ = false;

    if (dist < _waypoint_desired_dist_) {

      waypoint_reached_ = true;
      RCLCPP_INFO(node_->get_logger(), "waypoint reached");

      auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

      /* start idling at the reached waypoint */
      {
        mrs_lib::TimerHandlerOptions timer_opts_start;
        timer_opts_start.node      = node_;
        timer_opts_start.autostart = true;

        // makes the timer run only once
        timer_opts_start.oneshot = true;
        is_idling_               = true;

        std::function<void()> callback_fcn = std::bind(&WaypointFlier::timerIdling, this);

        timer_idling_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(1.0 / drs_params.waypoint_idle_time), callback_fcn);
      }

      RCLCPP_INFO(node_->get_logger(), "Idling for %f seconds.", drs_params.waypoint_idle_time);
    }
  }
}

//}

// | -------------------- timer callback ----------------------|

/* timerPublishSetReference() //{ */

void WaypointFlier::timerPublishSetReference() {

  if (!is_initialized_) {
    return;
  }

  /* return if the uav is still flying to the previous waypoints */
  if (have_goal_) {
    return;
  }

  /* return if the UAV is idling at a waypoint */
  if (is_idling_) {
    return;
  }

  /* shutdown node after flying through all the waypoints (call land service before) */
  if (idx_current_waypoint_ >= n_waypoints_) {

    c_loop_++;

    RCLCPP_INFO(node_->get_logger(), "finished loop %d/%d", c_loop_, _n_loops_);

    if (c_loop_ >= _n_loops_) {

      RCLCPP_INFO(node_->get_logger(), "finished %d loops of %d waypoints.", _n_loops_, n_waypoints_);

      if (_land_end_) {

        RCLCPP_INFO(node_->get_logger(), "calling land service.");

        std::shared_ptr<std_srvs::srv::Trigger::Request> req = std::make_shared<std_srvs::srv::Trigger::Request>();

        auto res = srv_client_land_.callSync(req);

        if (!res) {
          RCLCPP_ERROR(node_->get_logger(), "failed to call eland service");
        } else {
          if (!res.value()->success) {
            RCLCPP_ERROR(node_->get_logger(), "service call for eland failed, response: '%s'", res.value()->message.c_str());
          }
        }
      }

      timer_publisher_reference_->stop();

      return;

    } else {
      RCLCPP_INFO(node_->get_logger(), "starting loop %d/%d", c_loop_ + 1, _n_loops_);
      idx_current_waypoint_ = 0;
    }
  }

  /* create new waypoint msg */
  mrs_msgs::msg::ReferenceStamped new_waypoint;

  // set the frame id in which the reference is expressed
  new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
  new_waypoint.header.stamp    = clock_->now();

  new_waypoint.reference = waypoints_.at(idx_current_waypoint_);

  // set the variable under the mutex
  mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(idx_current_waypoint_), current_waypoint_);

  RCLCPP_INFO(node_->get_logger(), "Flying to waypoint %d: x: %.2f y: %.2f z: %.2f heading: %.2f", idx_current_waypoint_ + 1, new_waypoint.reference.position.x,
              new_waypoint.reference.position.y, new_waypoint.reference.position.z, new_waypoint.reference.heading);

  pub_reference_.publish(new_waypoint);

  if (waypoint_reached_) {
    idx_current_waypoint_++;
    waypoint_reached_ = false;
  }

  have_goal_ = true;
}

//}

/* timerPublishDistToWaypoint() //{ */

void WaypointFlier::timerPublishDistToWaypoint() {

  if (!is_initialized_) {
    return;
  }

  /* do not publish distance to waypoint when the uav is not flying towards a waypoint */
  if (!have_goal_) {
    return;
  }

  // this routine can not work without the odometry
  if (!sh_odometry_.hasMsg()) {
    return;
  }

  // get the variable under the mutex
  mrs_msgs::msg::Reference current_waypoint = mrs_lib::get_mutexed(mutex_current_waypoint_, current_waypoint_);

  // extract the pose part of the odometry
  geometry_msgs::msg::Pose current_pose = mrs_lib::getPose(sh_odometry_.getMsg());

  double dist = distance(current_waypoint, current_pose);
  RCLCPP_INFO(node_->get_logger(), "distance to waypoint is given is %.2f m", dist);

  mrs_msgs::msg::Float64Stamped dist_msg;

  // it is important to set the frame id correctly !!
  dist_msg.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
  dist_msg.header.stamp    = clock_->now();
  dist_msg.value           = dist;

  pub_dist_to_waypoint_.publish(dist_msg);
}

//}

/* timerCheckSubscribers() //{ */

void WaypointFlier::timerCheckSubscribers() {

  if (!is_initialized_) {
    return;
  }

  if (!sh_odometry_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Not received uav odom msg since node launch.");
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Not received tracker diagnostics msg since node launch.");
  }
}

//}

/* timerIdling() //{ */

void WaypointFlier::timerIdling() {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  clock_->sleep_for(std::chrono::duration<double>(drs_params.waypoint_idle_time));

  RCLCPP_INFO(node_->get_logger(), "Idling finished");
  is_idling_ = false;
}

//}

// | ------------------- service callbacks --------------------|

/* callbackStartWaypointFollowing() //{ */

bool WaypointFlier::callbackStartWaypointFollowing([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                   const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {

    response->success = false;
    response->message = "Waypoint flier not initialized!";
    RCLCPP_WARN(node_->get_logger(), "Cannot start waypoint following, nodelet is not initialized.");
    return true;
  }

  if (waypoints_loaded_) {

    timer_publisher_reference_->start();

    RCLCPP_INFO(node_->get_logger(), "Starting waypoint following.");

    response->success = true;
    response->message = "Starting waypoint following.";

  } else {

    RCLCPP_WARN(node_->get_logger(), "Cannot start waypoint following, waypoints are not set.");
    response->success = false;
    response->message = "Waypoints not set.";
  }

  return true;
}

//}

/* callbackStopWaypointFollowing() //{ */

bool WaypointFlier::callbackStopWaypointFollowing([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                  const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {

    response->success = false;
    response->message = "Waypoint flier not initialized!";
    RCLCPP_WARN(node_->get_logger(), "Cannot stop waypoint following, nodelet is not initialized.");
    return true;
  }

  timer_publisher_reference_->stop();

  RCLCPP_INFO(node_->get_logger(), "Waypoint following stopped.");

  response->success = true;
  response->message = "Waypoint following stopped.";

  return true;
}

//}

/* callbackFlyToFirstWaypoint() //{ */

bool WaypointFlier::callbackFlyToFirstWaypoint([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                               const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {

    response->success = false;
    response->message = "Waypoint flier not initialized!";
    RCLCPP_WARN(node_->get_logger(), "Cannot start waypoint following, nodelet is not initialized.");

    return true;
  }

  if (waypoints_loaded_) {

    /* create new waypoint msg */
    mrs_msgs::msg::ReferenceStamped new_waypoint;

    // it is important to set the frame id correctly !!
    new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
    new_waypoint.header.stamp    = clock_->now();
    new_waypoint.reference       = waypoints_.at(0);

    mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(0), current_waypoint_);

    // set the variable under the mutex

    idx_current_waypoint_ = 0;
    c_loop_               = 0;

    have_goal_ = true;

    try {
      pub_reference_.publish(new_waypoint);
    }
    catch (...) {
      RCLCPP_ERROR(node_->get_logger(), "Exception caught during publishing new waypoint");
    }

    std::stringstream ss;
    ss << "Flying to first waypoint: x: " << new_waypoint.reference.position.x << ", y: " << new_waypoint.reference.position.y
       << ", z: " << new_waypoint.reference.position.z << ", heading: " << new_waypoint.reference.heading;

    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    response->success = true;
    response->message = ss.str();

  } else {

    RCLCPP_WARN(node_->get_logger(), "Cannot fly to first waypoint, waypoints not loaded!");

    response->success = false;
    response->message = "Waypoints not loaded";
  }

  return true;
}

//}

// | ------------------- dynparam callbacks ------------------- |

/* callbackRatePublishDist() //{ */

void WaypointFlier::callbackRatePublishDist(const double param_value) {

  timer_publish_dist_to_waypoint_->setPeriod(rclcpp::Duration(std::chrono::duration<double>(1.0 / param_value)));

  RCLCPP_INFO(node_->get_logger(), "desired publisher rate updated to %.3f", param_value);
}

//}

// | -------------------- support functions ------------------- |

/* matrixToPoints() //{ */

std::vector<mrs_msgs::msg::Reference> WaypointFlier::matrixToPoints(const Eigen::MatrixXd& matrix) {

  std::vector<mrs_msgs::msg::Reference> points;

  for (int i = 0; i < matrix.rows(); i++) {

    mrs_msgs::msg::Reference point;
    point.position.x = matrix.row(i)(0);
    point.position.y = matrix.row(i)(1);
    point.position.z = matrix.row(i)(2);
    point.heading    = matrix.row(i)(3);

    points.push_back(point);
  }

  return points;
}

//}

/* offsetPoints() //{ */

void WaypointFlier::offsetPoints(std::vector<mrs_msgs::msg::Reference>& points, const Eigen::MatrixXd& offset) {

  for (size_t i = 0; i < points.size(); i++) {

    points.at(i).position.x += offset(0);
    points.at(i).position.y += offset(1);
    points.at(i).position.z += offset(2);
    points.at(i).heading += offset(3);
  }
}

//}

/* distance() //{ */

double WaypointFlier::distance(const mrs_msgs::msg::Reference& waypoint, const geometry_msgs::msg::Pose& pose) {

  return mrs_lib::geometry::dist(vec3_t(waypoint.position.x, waypoint.position.y, waypoint.position.z),
                                 vec3_t(pose.position.x, pose.position.y, pose.position.z));
}

//}

}  // namespace example_waypoint_flier

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(example_waypoint_flier::WaypointFlier)
