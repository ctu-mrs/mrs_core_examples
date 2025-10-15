/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/tracker.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/geometry/cyclic.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

//}

namespace example_tracker_plugin
{

/* DrsParams_t //{ */

struct DrsParams_t
{
  double z_offset;
};

//}

/* //{ class ExampleTracker */

class ExampleTracker : public mrs_uav_managers::Tracker {

public:
  bool initialize(const rclcpp::Node::SharedPtr& node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  std::tuple<bool, std::string> activate([[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand>& last_tracker_cmd);
  void                          deactivate(void);
  bool                          resetStatic(void);
  void                          destroy(void);

  std::optional<mrs_msgs::msg::TrackerCommand>            update(const mrs_msgs::msg::UavState&                     uav_state,
                                                                 const mrs_uav_managers::Controller::ControlOutput& last_control_output);
  const mrs_msgs::msg::TrackerStatus                      getStatus();
  const std::shared_ptr<std_srvs::srv::SetBool::Response> enableCallbacks(const std::shared_ptr<std_srvs::srv::SetBool::Request>& request);
  const std::shared_ptr<std_srvs::srv::Trigger::Response> switchOdometrySource(const mrs_msgs::msg::UavState& new_uav_state);

  const std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Response>         setReference(const std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Request>& request);
  const std::shared_ptr<mrs_msgs::srv::VelocityReferenceSrv::Response> setVelocityReference(
      const std::shared_ptr<mrs_msgs::srv::VelocityReferenceSrv::Request>& request);
  const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Response> setTrajectoryReference(
      const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Request>& request);

  const std::shared_ptr<std_srvs::srv::Trigger::Response> hover(const std::shared_ptr<std_srvs::srv::Trigger::Request>& request);
  const std::shared_ptr<std_srvs::srv::Trigger::Response> startTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request>& request);
  const std::shared_ptr<std_srvs::srv::Trigger::Response> stopTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request>& request);
  const std::shared_ptr<std_srvs::srv::Trigger::Response> resumeTrajectoryTracking(const std::shared_ptr<std_srvs::srv::Trigger::Request>& request);
  const std::shared_ptr<std_srvs::srv::Trigger::Response> gotoTrajectoryStart(const std::shared_ptr<std_srvs::srv::Trigger::Request>& request);

  const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response> setConstraints(
      const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request>& request);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  bool callbacks_enabled_ = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  std::string             _uav_name;
  mrs_msgs::msg::UavState uav_state_;
  std::mutex              mutex_uav_state_;

  // | ----------------------- parameters ----------------------- |

  std::string _run_type_;

  // | --------------- dynamic reconfigure server --------------- |

  std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;
  std::mutex                            mutex_dynamic_reconfigure_;
  DrsParams_t                           drs_params_;

  // | ------------------ dynamics constriants ------------------ |

  mrs_msgs::msg::DynamicsConstraints constraints_;
  std::mutex                         mutex_constraints_;

  // | ----------------------- goal state ----------------------- |

  double goal_x_       = 0;
  double goal_y_       = 0;
  double goal_z_       = 0;
  double goal_heading_ = 0;

  // | ---------------- the tracker's inner state --------------- |

  std::atomic<bool> is_initialized_ = false;
  std::atomic<bool> is_active_      = false;

  double pos_x_   = 0;
  double pos_y_   = 0;
  double pos_z_   = 0;
  double heading_ = 0;

  // | ------------------- for calculating dt ------------------- |

  rclcpp::Time      last_update_time_;
  std::atomic<bool> first_iteration_ = true;

  void callbackDrs(const std::string param_name, [[maybe_unused]] uint32_t level);
};

//}

// | ------------------- trackers interface ------------------- |

/* initialize() //{ */

bool ExampleTracker::initialize(const rclcpp::Node::SharedPtr& node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  this->common_handlers_  = common_handlers;
  this->private_handlers_ = private_handlers;

  node_  = node;
  clock_ = node_->get_clock();

  _uav_name = common_handlers->uav_name;

  last_update_time_ = clock_->now();

  // |--------------------- load plugins parameters ---------------------|

  // add yaml config from the package's config folder
  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("example_tracker_plugin") + "/config/example_tracker.yaml");

  // setup the dynamical reconfigure parameter manager
  dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_dynamic_reconfigure_);

  // copy loaded yaml files from the param loader to the dynparam mgr
  dynparam_mgr_->get_param_provider().copyYamls(private_handlers->param_loader->getParamProvider());

  // prepare a dynamically-reconfigurable parameter and load the default from yaml
  // the parameter is loaded from the namespace of the plugin
  //    ... that is mrs_uav_managers/example_tracker
  dynparam_mgr_->register_param("z_offset", &drs_params_.z_offset, mrs_lib::DynparamMgr::range_t<double>(-1.0, 1.0));

  // load a traditional parameter
  int other_parameter;
  private_handlers->param_loader->loadParam("different_parameter", other_parameter);

  // load a traditional parameter from the namespace of the control manager
  // btw, run_type can be {"simulation", "realworld"}
  private_handlers->parent_param_loader->loadParam("run_type", _run_type_);

  if (!private_handlers->param_loader->loadedSuccessfully() || !dynparam_mgr_->loaded_successfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    return false;
  }

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");

  return true;
}

//}

/* activate() //{ */

std::tuple<bool, std::string> ExampleTracker::activate([[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand>& last_tracker_cmd) {

  if (last_tracker_cmd) {

    // actually, you should actually check if these entries are filled in
    pos_x_   = last_tracker_cmd->position.x;
    pos_y_   = last_tracker_cmd->position.y;
    pos_z_   = last_tracker_cmd->position.z;
    heading_ = last_tracker_cmd->heading;

    goal_x_       = pos_x_;
    goal_y_       = pos_y_;
    goal_z_       = pos_z_;
    goal_heading_ = heading_;

  } else {

    auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

    pos_x_ = uav_state.pose.position.x;
    pos_y_ = uav_state.pose.position.y;
    pos_z_ = uav_state.pose.position.z;

    try {
      heading_ = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
    }
    catch (...) {
      RCLCPP_WARN(node_->get_logger(), "could not calculate heading from uav_state.pose.orientation");
      heading_ = 0;
    }
  }

  std::stringstream ss;
  ss << "activated";

  RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());
  is_active_ = true;

  return std::tuple(true, ss.str());
}

//}

/* deactivate() //{ */

void ExampleTracker::deactivate(void) {

  is_active_ = false;
  RCLCPP_INFO(node_->get_logger(), "deactivated");
}

//}

/* destroy() //{ */

void ExampleTracker::destroy(void) {

  is_active_ = false;
  RCLCPP_INFO(node_->get_logger(), "destroyed");
}

//}

/* switchOdometrySource() //{ */

const std::shared_ptr<std_srvs::srv::Trigger::Response> ExampleTracker::switchOdometrySource([[maybe_unused]] const mrs_msgs::msg::UavState& new_uav_state) {

  // This tracker does not implement this method, which is ok.
  // Returning nullptr will "signal" to the Control Manager that nothing will happen.

  return nullptr;
}

//}

/* update() //{ */

std::optional<mrs_msgs::msg::TrackerCommand> ExampleTracker::update([[maybe_unused]] const mrs_msgs::msg::UavState&                     uav_state,
                                                                    [[maybe_unused]] const mrs_uav_managers::Controller::ControlOutput& last_control_output) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);
  auto drs_params  = mrs_lib::get_mutexed(mutex_dynamic_reconfigure_, drs_params_);

  // |---------- calculate dt from the last iteration ---------|
  double dt;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = (rclcpp::Time(uav_state.header.stamp).seconds() - last_update_time_.seconds());
  }

  last_update_time_ = rclcpp::Time(uav_state.header.stamp);

  if (fabs(dt) < 0.001) {
    RCLCPP_DEBUG(node_->get_logger(), "the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }

  // up to this part the update() method is evaluated even when the tracker is not active
  if (!is_active_) {
    return {};
  }

  Eigen::Vector2d vec_to_goal_horizontal(goal_x_ - pos_x_, goal_y_ - pos_y_);
  double          to_goal_vertical = goal_z_ - pos_z_;
  double          to_goal_heading  = mrs_lib::geometry::sradians::diff(goal_heading_, heading_);

  Eigen::Vector2d step_horizontal = vec_to_goal_horizontal.normalized() * constraints.horizontal_speed * dt;

  double step_vertical;

  if (to_goal_vertical >= 0) {
    step_vertical = constraints.vertical_ascending_speed * dt;
  } else {
    step_vertical = -constraints.vertical_descending_speed * dt;
  }

  double step_heading = mrs_lib::signum(to_goal_heading) * constraints.heading_speed * dt;

  pos_x_ += step_horizontal(0);
  pos_y_ += step_horizontal(1);
  pos_z_ += step_vertical;
  heading_ += step_heading;

  // | --------------- check for reaching the goal -------------- |

  if (vec_to_goal_horizontal.norm() <= constraints.horizontal_speed * dt) {
    pos_x_ = goal_x_;
    pos_y_ = goal_y_;
  }

  if (to_goal_vertical > 0 && to_goal_vertical <= constraints.vertical_ascending_speed * dt) {
    pos_z_ = goal_z_;
  }

  if (to_goal_vertical < 0 && -to_goal_vertical <= constraints.vertical_descending_speed * dt) {
    pos_z_ = goal_z_;
  }

  if (std::abs(to_goal_heading) <= constraints.heading_speed * dt) {
    heading_ = goal_heading_;
  }

  // | ------------------- fill in the result ------------------- |

  mrs_msgs::msg::TrackerCommand tracker_cmd;

  tracker_cmd.header.stamp    = clock_->now();
  tracker_cmd.header.frame_id = uav_state.header.frame_id;

  tracker_cmd.position.x = pos_x_;
  tracker_cmd.position.y = pos_y_;
  tracker_cmd.position.z = pos_z_ + drs_params.z_offset;  // let's add the dynamically-reconfigurable z_offset
  tracker_cmd.heading    = heading_;

  tracker_cmd.use_position_vertical   = 1;
  tracker_cmd.use_position_horizontal = 1;
  tracker_cmd.use_heading             = 1;

  return {tracker_cmd};
}

//}

/* resetStatic() //{ */

bool ExampleTracker::resetStatic(void) {

  return false;
}

//}

/* getStatus() //{ */

const mrs_msgs::msg::TrackerStatus ExampleTracker::getStatus() {

  mrs_msgs::msg::TrackerStatus tracker_status;

  tracker_status.active            = is_active_;
  tracker_status.callbacks_enabled = callbacks_enabled_;

  return tracker_status;
}

//}

/* enableCallbacks() //{ */

const std::shared_ptr<std_srvs::srv::SetBool::Response> ExampleTracker::enableCallbacks(const std::shared_ptr<std_srvs::srv::SetBool::Request>& request) {

  std::shared_ptr<std_srvs::srv::SetBool::Response> response = std::make_shared<std_srvs::srv::SetBool::Response>();

  std::stringstream ss;

  if (request->data != callbacks_enabled_) {

    callbacks_enabled_ = request->data;

    ss << "callbacks " << (callbacks_enabled_ ? "enabled" : "disabled");
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "[LandoffTrakcer]: " << ss.str());

  } else {

    ss << "callbacks were already " << (callbacks_enabled_ ? "enabled" : "disabled");
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "[LandoffTrakcer]: " << ss.str());
  }

  response->message = ss.str();
  response->success = true;

  return response;
}

//}

/* setReference() //{ */

const std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Response> ExampleTracker::setReference(
    [[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::ReferenceSrv::Request>& request) {

  goal_x_       = request->reference.position.x;
  goal_y_       = request->reference.position.y;
  goal_z_       = request->reference.position.z;
  goal_heading_ = request->reference.heading;

  mrs_msgs::srv::ReferenceSrv_Response response;

  response.message = "reference set";
  response.success = true;

  return std::shared_ptr<mrs_msgs::srv::ReferenceSrv_Response>(new mrs_msgs::srv::ReferenceSrv_Response(response));
}

//}

/* setVelocityReference() //{ */

const std::shared_ptr<mrs_msgs::srv::VelocityReferenceSrv::Response> ExampleTracker::setVelocityReference(
    [[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::VelocityReferenceSrv::Request>& request) {

  // This tracker does not implement this method, which is ok.
  // Returning nullptr will "signal" to the Control Manager that nothing will happen.

  return nullptr;
}

//}

/* setTrajectoryReference() //{ */

const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Response> ExampleTracker::setTrajectoryReference(
    [[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Request>& request) {

  // This tracker does not implement this method, which is ok.
  // Returning nullptr will "signal" to the Control Manager that nothing will happen.

  return nullptr;
}

//}

/* hover() //{ */

const std::shared_ptr<std_srvs::srv::Trigger::Response> ExampleTracker::hover(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>& request) {

  // This tracker does not implement this method, which is ok.
  // Returning nullptr will "signal" to the Control Manager that nothing will happen.

  return nullptr;
}

//}

/* startTrajectoryTracking() //{ */

const std::shared_ptr<std_srvs::srv::Trigger::Response> ExampleTracker::startTrajectoryTracking(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>& request) {

  // This tracker does not implement this method, which is ok.
  // Returning nullptr will "signal" to the Control Manager that nothing will happen.

  return nullptr;
}

//}

/* stopTrajectoryTracking() //{ */

const std::shared_ptr<std_srvs::srv::Trigger::Response> ExampleTracker::stopTrajectoryTracking(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>& request) {

  // This tracker does not implement this method, which is ok.
  // Returning nullptr will "signal" to the Control Manager that nothing will happen.

  return nullptr;
}

//}

/* resumeTrajectoryTracking() //{ */

const std::shared_ptr<std_srvs::srv::Trigger::Response> ExampleTracker::resumeTrajectoryTracking(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>& request) {

  // This tracker does not implement this method, which is ok.
  // Returning nullptr will "signal" to the Control Manager that nothing will happen.

  return nullptr;
}

//}

/* gotoTrajectoryStart() //{ */

const std::shared_ptr<std_srvs::srv::Trigger::Response> ExampleTracker::gotoTrajectoryStart(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>& request) {

  // This tracker does not implement this method, which is ok.
  // Returning nullptr will "signal" to the Control Manager that nothing will happen.

  return nullptr;
}

//}

/* setConstraints() //{ */

const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response> ExampleTracker::setConstraints(
    [[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request>& request) {

  mrs_lib::set_mutexed(mutex_constraints_, request->constraints, constraints_);

  mrs_msgs::srv::DynamicsConstraintsSrv_Response res;

  res.success = true;
  res.message = "constraints updated";

  return std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv_Response>(new mrs_msgs::srv::DynamicsConstraintsSrv_Response(res));
}

//}

}  // namespace example_tracker_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(example_tracker_plugin::ExampleTracker, mrs_uav_managers::Tracker)
