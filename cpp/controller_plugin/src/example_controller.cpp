/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <pid.hpp>

#include <mrs_uav_managers/controller.h>

#include <dynamic_reconfigure/server.h>
#include <example_controller_plugin/example_controllerConfig.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

//}

namespace example_controller_plugin
{

namespace example_controller
{

/* //{ class ExampleController */

class ExampleController : public mrs_uav_managers::Controller {

public:
  bool initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput& last_control_output);

  void deactivate(void);

  void updateInactive(const mrs_msgs::UavState& uav_state, const std::optional<mrs_msgs::TrackerCommand>& tracker_command);

  ControlOutput updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState& new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& cmd);

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                      mutex_drs_;
  typedef example_controller_plugin::example_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>            Drs_t;
  boost::shared_ptr<Drs_t>                                    drs_;
  void                                                        callbackDrs(example_controller_plugin::example_controllerConfig& config, uint32_t level);
  DrsConfig_t                                                 drs_params_;
  std::mutex                                                  mutex_drs_params_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

  // | --------- throttle generation and mass estimation -------- |

  double _uav_mass_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

bool ExampleController::initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                   std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  nh_ = nh;

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  last_update_time_ = ros::Time(0);

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  bool success = true;

  // FYI
  // This method will load the file using `rosparam get`
  //   Pros: you can the full power of the official param loading
  //   Cons: it is slower
  //
  // Alternatives:
  //   You can load the file directly into the ParamLoader as shown below.

  success *= private_handlers->loadConfigFile(ros::package::getPath("example_controller_plugin") + "/config/example_controller.yaml");

  if (!success) {
    return false;
  }

  mrs_lib::ParamLoader param_loader(nh_, "ExampleController");

  // This is the alternaive way of loading the config file.
  //
  // Files loaded using this method are prioritized over ROS params.
  //
  // param_loader.addYamlFile(ros::package::getPath("example_tracker_plugin") + "/config/example_tracker.yaml");

  param_loader.loadParam("desired_roll", drs_params_.roll);
  param_loader.loadParam("desired_pitch", drs_params_.pitch);
  param_loader.loadParam("desired_yaw", drs_params_.yaw);
  param_loader.loadParam("desired_thrust_force", drs_params_.force);

  // | ------------------ finish loading params ----------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ExampleController]: could not load all parameters!");
    return false;
  }

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&ExampleController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[ExampleController]: initialized");

  is_initialized_ = true;

  return true;
}

//}

/* //{ activate() */

bool ExampleController::activate(const ControlOutput& last_control_output) {

  activation_control_output_ = last_control_output;

  first_iteration_ = true;

  is_active_ = true;

  ROS_INFO("[ExampleController]: activated");

  return true;
}

//}

/* //{ deactivate() */

void ExampleController::deactivate(void) {

  is_active_       = false;
  first_iteration_ = false;

  ROS_INFO("[ExampleController]: deactivated");
}

//}

/* updateInactive() //{ */

void ExampleController::updateInactive(const mrs_msgs::UavState& uav_state, [[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand>& tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateActive() */

ExampleController::ControlOutput ExampleController::updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  // clear all the optional parts of the result
  last_control_output_.desired_heading_rate          = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.control_output                = {};

  if (!is_active_) {
    return last_control_output_;
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

    ROS_DEBUG("[ExampleController]: the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }

  // | -------- check for the available output modalities ------- |

  // you can decide what to return, but it needs to be available
  if (common_handlers_->control_output_modalities.attitude) {
    ROS_INFO_THROTTLE(1.0, "[ExampleController]: desired attitude output modality is available");
  }

  // | ---------- extract the detailed model parameters --------- |

  if (common_handlers_->detailed_model_params) {

    mrs_uav_managers::control_manager::DetailedModelParams_t detailed_model_params = common_handlers_->detailed_model_params.value();

    ROS_INFO_STREAM_THROTTLE(1.0, "[ExampleController]: UAV inertia is: " << detailed_model_params.inertia);
  }

  // | -------------- prepare the control reference ------------- |

  geometry_msgs::PoseStamped position_reference;

  position_reference.header           = tracker_command.header;
  position_reference.pose.position    = tracker_command.position;
  position_reference.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(tracker_command.heading);


  // | ---------------- prepare the control output --------------- |

  mrs_msgs::HwApiAttitudeCmd attitude_cmd;

  attitude_cmd.orientation = mrs_lib::AttitudeConverter(drs_params.roll, drs_params.pitch, drs_params.yaw);
  attitude_cmd.throttle    = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model,
                                                                             common_handlers_->getMass() * common_handlers_->g + drs_params.force);


  // | ----------------- set the control output ----------------- |

  last_control_output_.control_output = attitude_cmd;

  // | --------------- fill in the optional parts --------------- |

  //// it is recommended to fill the optinal parts if you know them

  /// this is used for:
  // * plotting the orientation in the control_refence topic (optional)
  // * checking for attitude control error
  last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(drs_params.roll, drs_params.pitch, drs_params.yaw);

  /// IMPORANT
  // The acceleration and heading rate in 3D (expressed in the "fcu" frame of reference) that the UAV will actually undergo due to the control action.
  last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(0, 0, 0);
  last_control_output_.desired_heading_rate          = 0;

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.controller = "ExampleController";

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus ExampleController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void ExampleController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState& new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void ExampleController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr ExampleController::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& constraints) {

  if (!is_initialized_) {
    return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  ROS_INFO("[ExampleController]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void ExampleController::callbackDrs(example_controller_plugin::example_controllerConfig& config, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_drs_params_, config, drs_params_);

  ROS_INFO("[ExampleController]: dynamic reconfigure params updated");
}

//}

}  // namespace example_controller

}  // namespace example_controller_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(example_controller_plugin::example_controller::ExampleController, mrs_uav_managers::Controller)
