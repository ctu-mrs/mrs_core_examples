/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_uav_managers/tracker.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>

#include <dynamic_reconfigure/server.h>
#include <example_tracker_plugin/example_trackerConfig.h>

//}

namespace example_tracker_plugin
{

namespace example_tracker
{

/* //{ class ExampleTracker */

class ExampleTracker : public mrs_uav_managers::Tracker {
public:
  bool initialize(const ros::NodeHandle &nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  std::tuple<bool, std::string> activate(const std::optional<mrs_msgs::TrackerCommand> &last_tracker_cmd);
  void                          deactivate(void);
  bool                          resetStatic(void);

  std::optional<mrs_msgs::TrackerCommand>   update(const mrs_msgs::UavState &uav_state, const mrs_uav_managers::Controller::ControlOutput &last_control_output);
  const mrs_msgs::TrackerStatus             getStatus();
  const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr switchOdometrySource(const mrs_msgs::UavState &new_uav_state);

  const mrs_msgs::ReferenceSrvResponse::ConstPtr           setReference(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::VelocityReferenceSrvResponse::ConstPtr   setVelocityReference(const mrs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr setTrajectoryReference(const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

  const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd);

private:
  ros::NodeHandle nh_;

  bool callbacks_enabled_ = true;

  std::string _uav_name_;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  bool               got_uav_state_ = false;
  std::mutex         mutex_uav_state_;

  // | ------------------ dynamics constriants ------------------ |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

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

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                mutex_drs_;
  typedef example_tracker_plugin::example_trackerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>      Drs_t;
  boost::shared_ptr<Drs_t>                              drs_;
  void                                                  callbackDrs(example_tracker_plugin::example_trackerConfig &config, uint32_t level);
  DrsConfig_t                                           drs_params_;
  std::mutex                                            mutex_drs_params_;
};

//}

// | -------------- tracker's interface routines -------------- |

/* //{ initialize() */

bool ExampleTracker::initialize(const ros::NodeHandle &nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  this->common_handlers_  = common_handlers;
  this->private_handlers_ = private_handlers;

  _uav_name_ = common_handlers->uav_name;

  nh_ = nh;

  ros::Time::waitForValid();

  last_update_time_ = ros::Time(0);

  // --------------------------------------------------------------
  // |                     loading parameters                     |
  // --------------------------------------------------------------

  // | -------------------- load param files -------------------- |

  bool success = true;

  // FYI
  // This method will load the file using `rosparam get`
  //   Pros: you can the full power of the official param loading
  //   Cons: it is slower
  //
  // Alternatives:
  //   You can load the file directly into the ParamLoader as shown below.

  success *= private_handlers->loadConfigFile(ros::package::getPath("example_tracker_plugin") + "/config/example_tracker.yaml");

  if (!success) {
    return false;
  }

  // | ---------------- load plugin's parameters ---------------- |

  mrs_lib::ParamLoader param_loader(nh_, "ExampleTracker");

  // This is the alternaive way of loading the config file.
  //
  // Files loaded using this method are prioritized over ROS params.
  //
  // param_loader.addYamlFile(ros::package::getPath("example_tracker_plugin") + "/config/example_tracker.yaml");

  param_loader.loadParam("some_parameter", drs_params_.some_parameter);


  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ExampleTracker]: could not load all parameters!");
    return false;
  }

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&ExampleTracker::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO("[ExampleTracker]: initialized");

  return true;
}

//}

/* //{ activate() */

std::tuple<bool, std::string> ExampleTracker::activate([[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand> &last_tracker_cmd) {

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
      heading_ = 0;
    }
  }

  std::stringstream ss;
  ss << "activated";
  ROS_INFO_STREAM("[ExampleTracker]: " << ss.str());

  is_active_ = true;

  return std::tuple(true, ss.str());
}

//}

/* //{ deactivate() */

void ExampleTracker::deactivate(void) {

  is_active_ = false;

  ROS_INFO("[ExampleTracker]: deactivated");
}

//}

/* //{ update() */

std::optional<mrs_msgs::TrackerCommand> ExampleTracker::update(const mrs_msgs::UavState &                                          uav_state,
                                                               [[maybe_unused]] const mrs_uav_managers::Controller::ControlOutput &last_control_output) {

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = uav_state;

    got_uav_state_ = true;
  }

  // | ---------- calculate dt from the last iteration ---------- |

  double dt;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = (uav_state.header.stamp - last_update_time_).toSec();
  }

  last_update_time_ = uav_state.header.stamp;

  if (fabs(dt) < 0.001) {

    ROS_DEBUG("[ExampleTracker]: the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }

  // up to this part the update() method is evaluated even when the tracker is not active
  if (!is_active_) {
    return {};
  }

  // | ------------------ move the inner state ------------------ |

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

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

  mrs_msgs::TrackerCommand tracker_cmd;

  tracker_cmd.header.stamp    = ros::Time::now();
  tracker_cmd.header.frame_id = uav_state.header.frame_id;

  tracker_cmd.position.x = pos_x_;
  tracker_cmd.position.y = pos_y_;
  tracker_cmd.position.z = pos_z_;
  tracker_cmd.heading    = heading_;

  tracker_cmd.use_position_vertical   = 1;
  tracker_cmd.use_position_horizontal = 1;
  tracker_cmd.use_heading             = 1;

  return {tracker_cmd};
}

//}

/* //{ resetStatic() */

bool ExampleTracker::resetStatic(void) {

  return false;
}

//}

/* //{ getStatus() */

const mrs_msgs::TrackerStatus ExampleTracker::getStatus() {

  mrs_msgs::TrackerStatus tracker_status;

  tracker_status.active            = is_active_;
  tracker_status.callbacks_enabled = callbacks_enabled_;

  return tracker_status;
}

//}

/* //{ enableCallbacks() */

const std_srvs::SetBoolResponse::ConstPtr ExampleTracker::enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) {

  std_srvs::SetBoolResponse res;
  std::stringstream         ss;

  if (cmd->data != callbacks_enabled_) {

    callbacks_enabled_ = cmd->data;

    ss << "callbacks " << (callbacks_enabled_ ? "enabled" : "disabled");
    ROS_INFO_STREAM_THROTTLE(1.0, "[ExampleTracker]: " << ss.str());

  } else {

    ss << "callbacks were already " << (callbacks_enabled_ ? "enabled" : "disabled");
    ROS_WARN_STREAM_THROTTLE(1.0, "[ExampleTracker]: " << ss.str());
  }

  res.message = ss.str();
  res.success = true;

  return std_srvs::SetBoolResponse::ConstPtr(new std_srvs::SetBoolResponse(res));
}

//}

/* switchOdometrySource() //{ */

const std_srvs::TriggerResponse::ConstPtr ExampleTracker::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState &new_uav_state) {

  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ hover() */

const std_srvs::TriggerResponse::ConstPtr ExampleTracker::hover([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {

  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ startTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr ExampleTracker::startTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ stopTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr ExampleTracker::stopTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ resumeTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr ExampleTracker::resumeTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ gotoTrajectoryStart() */

const std_srvs::TriggerResponse::ConstPtr ExampleTracker::gotoTrajectoryStart([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ setConstraints() */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr ExampleTracker::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd) {

  {
    std::scoped_lock lock(mutex_constraints_);

    constraints_ = cmd->constraints;
  }

  mrs_msgs::DynamicsConstraintsSrvResponse res;

  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

/* //{ setReference() */

const mrs_msgs::ReferenceSrvResponse::ConstPtr ExampleTracker::setReference([[maybe_unused]] const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {

  goal_x_       = cmd->reference.position.x;
  goal_y_       = cmd->reference.position.y;
  goal_z_       = cmd->reference.position.z;
  goal_heading_ = cmd->reference.heading;

  mrs_msgs::ReferenceSrvResponse response;

  response.message = "reference set";
  response.success = true;

  return mrs_msgs::ReferenceSrvResponse::ConstPtr(new mrs_msgs::ReferenceSrvResponse(response));
}

//}

/* //{ setVelocityReference() */

const mrs_msgs::VelocityReferenceSrvResponse::ConstPtr ExampleTracker::setVelocityReference([
    [maybe_unused]] const mrs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::VelocityReferenceSrvResponse::Ptr();
}

//}

/* //{ setTrajectoryReference() */

const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr ExampleTracker::setTrajectoryReference([
    [maybe_unused]] const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::TrajectoryReferenceSrvResponse::Ptr();
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void ExampleTracker::callbackDrs(example_tracker_plugin::example_trackerConfig &config, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_drs_params_, config, drs_params_);

  ROS_INFO("[Exampletracker]: dynamic reconfigure params updated");
}

//}

}  // namespace example_tracker

}  // namespace example_tracker_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(example_tracker_plugin::example_tracker::ExampleTracker, mrs_uav_managers::Tracker)
