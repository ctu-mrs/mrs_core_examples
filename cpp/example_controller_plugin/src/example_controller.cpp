/* includes //{*/

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/controller.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/dynparam_mgr.h>

#include <pid.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

//}

namespace example_controller_plugin
{

/* DrsParams_t //{ */

struct DrsParams_t
{
  double gain_xyz_p;
  double gain_xyz_i;
  double gain_xyz_d;
  double gain_heading_p;
  double gain_heading_i;
  double gain_heading_d;
};

//}

/* //{ class ExampleController */

class ExampleController : public mrs_uav_managers::Controller {

public:
  bool initialize(const rclcpp::Node::SharedPtr& node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput& last_control_output);

  void deactivate(void);

  void destroy();

  void updateInactive(const mrs_msgs::msg::UavState& uav_state, const std::optional<mrs_msgs::msg::TrackerCommand>& tracker_command);

  ControlOutput updateActive(const mrs_msgs::msg::UavState& uav_state, const mrs_msgs::msg::TrackerCommand& tracker_command);

  const mrs_msgs::msg::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::msg::UavState& new_uav_state);

  void resetDisturbanceEstimators(void);

  const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response> setConstraints(
      const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request>& constraints);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------- params ------------------------- |

  bool _antiwindup_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::msg::UavState uav_state_;
  std::mutex              mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;
  std::mutex                            mutex_dynparam_mgr_;
  DrsParams_t                           drs_params_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::msg::DynamicsConstraints constraints_;
  std::mutex                         mutex_constraints_;

  // | --------- throttle generation and mass estimation -------- |

  double _uav_mass_;

  // | --------------------------- PID -------------------------- |

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
  PIDController pid_heading_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  rclcpp::Time      last_update_time_;
  std::atomic<bool> first_iteration_ = true;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

bool ExampleController::initialize(const rclcpp::Node::SharedPtr& node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                   std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  node_  = node;
  clock_ = node_->get_clock();

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  last_update_time_ = clock_->now();

  // | --------------------- load parameters -------------------- |

  // add yaml config from the package's config folder
  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("example_controller_plugin") + "/config/example_controller.yaml");

  // setup the dynamical reconfigure parameter manager
  dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_dynparam_mgr_);

  // copy loaded yaml files from the param loader to the dynparam mgr
  dynparam_mgr_->get_param_provider().copyYamls(private_handlers->param_loader->getParamProvider());

  // the parameter is loaded from the namespace of the plugin
  //    ... that is mrs_uav_managers/example_controller
  private_handlers->param_loader->loadParam("antiwindup", _antiwindup_);

  // setup dynamically-reconfigurable parameters (gains)
  dynparam_mgr_->register_param("pid_gains/xyz/p", &drs_params_.gain_xyz_p, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_gains/xyz/i", &drs_params_.gain_xyz_i, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_gains/xyz/d", &drs_params_.gain_xyz_d, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_gains/heading/p", &drs_params_.gain_heading_p, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_gains/heading/i", &drs_params_.gain_heading_i, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("pid_gains/heading/d", &drs_params_.gain_heading_d, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));

  // | ----------------------- prepare PID ---------------------- |

  pid_x_.setParams(drs_params_.gain_xyz_p, drs_params_.gain_xyz_i, drs_params_.gain_xyz_d, 1.0, _antiwindup_);
  pid_y_.setParams(drs_params_.gain_xyz_p, drs_params_.gain_xyz_i, drs_params_.gain_xyz_d, 1.0, _antiwindup_);
  pid_z_.setParams(drs_params_.gain_xyz_p, drs_params_.gain_xyz_i, drs_params_.gain_xyz_d, 1.0, _antiwindup_);
  pid_heading_.setParams(drs_params_.gain_heading_p, drs_params_.gain_heading_i, drs_params_.gain_heading_d, 1.0, _antiwindup_);

  // | ------------------ finish loading params ----------------- |

  if (!private_handlers->param_loader->loadedSuccessfully() || !dynparam_mgr_->loaded_successfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    return false;
  }

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;

  return true;
}

//}

/* //{ activate() */

bool ExampleController::activate(const ControlOutput& last_control_output) {

  activation_control_output_ = last_control_output;

  first_iteration_ = true;
  is_active_       = true;

  RCLCPP_INFO(node_->get_logger(), "activated");

  return true;
}

//}

/* //{ deactivate() */

void ExampleController::deactivate(void) {

  is_active_       = false;
  first_iteration_ = false;

  RCLCPP_INFO(node_->get_logger(), "deactivated");
}

//}

/* //{ destroy() */

void ExampleController::destroy(void) {

  is_active_       = false;
  first_iteration_ = false;

  RCLCPP_INFO(node_->get_logger(), "destroyed");
}

//}

/* updateInactive() //{ */

void ExampleController::updateInactive(const mrs_msgs::msg::UavState&                                       uav_state,
                                       [[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand>& tracker_command) {

  mrs_lib::set_mutexed(mutex_dynparam_mgr_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateActive() */

ExampleController::ControlOutput ExampleController::updateActive(const mrs_msgs::msg::UavState&       uav_state,
                                                                 const mrs_msgs::msg::TrackerCommand& tracker_command) {

  auto drs_params  = mrs_lib::get_mutexed(mutex_dynparam_mgr_, drs_params_);
  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

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
    dt = rclcpp::Time(uav_state.header.stamp).seconds() - last_update_time_.seconds();
  }

  last_update_time_ = rclcpp::Time(uav_state.header.stamp);

  if (fabs(dt) < 0.001) {

    RCLCPP_WARN(node_->get_logger(), "the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }

  // | -------- check for the available output modalities ------- |

  // you can decide what to return, but it needs to be available
  if (common_handlers_->control_output_modalities.attitude) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "desired attitude output modality is available");
  }

  // | ---------- extract the detailed model parameters --------- |

  if (common_handlers_->detailed_model_params) {

    mrs_uav_managers::control_manager::DetailedModelParams_t detailed_model_params = common_handlers_->detailed_model_params.value();

    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "UAV inertia is: " << detailed_model_params.inertia);
  }

  // | -------------- prepare the control reference ------------- |

  geometry_msgs::msg::PoseStamped position_reference;

  position_reference.header           = tracker_command.header;
  position_reference.pose.position    = tracker_command.position;
  position_reference.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(tracker_command.heading);

  // | ----------------- prepare control errors ----------------- |

  double uav_heading = 0;

  {
    try {
      uav_heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
    }
    catch (...) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "failed to calculate UAV's heading");

      // let's set this to the reference, this will cause 0 control error and therefore no wierd control action
      uav_heading = tracker_command.heading;
    }
  }

  double e_x   = tracker_command.position.x - uav_state.pose.position.x;
  double e_y   = tracker_command.position.y - uav_state.pose.position.y;
  double e_z   = tracker_command.position.z - uav_state.pose.position.z;
  double e_hdg = mrs_lib::geometry::sradians::diff(tracker_command.heading, uav_heading);

  // | --------------- update the PIDs parameters --------------- |

  pid_x_.setParams(drs_params.gain_xyz_p, drs_params.gain_xyz_i, drs_params.gain_xyz_d, constraints.horizontal_acceleration, _antiwindup_);
  pid_y_.setParams(drs_params.gain_xyz_p, drs_params.gain_xyz_i, drs_params.gain_xyz_d, constraints.horizontal_acceleration, _antiwindup_);

  double max_vert_acc = constraints.vertical_descending_acceleration < constraints.vertical_ascending_acceleration
                            ? constraints.vertical_descending_acceleration
                            : constraints.vertical_ascending_acceleration;

  pid_z_.setParams(drs_params.gain_xyz_p, drs_params.gain_xyz_i, drs_params.gain_xyz_d, max_vert_acc, _antiwindup_);

  // heading rate IS NOT yaw rate...
  // but in this example, considering we are flying near-hover, let's constrain it by yaw rate
  pid_heading_.setParams(drs_params.gain_heading_p, drs_params.gain_heading_i, drs_params.gain_heading_d, constraints.yaw_rate, _antiwindup_);

  // | --------------- calculate the control input -------------- |

  mrs_msgs::msg::HwApiAccelerationHdgRateCmd cmd;

  cmd.acceleration.x = pid_x_.update(e_x, dt);
  cmd.acceleration.y = pid_y_.update(e_y, dt);
  cmd.acceleration.z = pid_z_.update(e_z, dt);
  cmd.heading_rate   = pid_heading_.update(e_hdg, dt);

  // | --------------------- add feedforward -------------------- |

  if (tracker_command.use_acceleration) {
    cmd.acceleration.x += tracker_command.acceleration.x;
    cmd.acceleration.y += tracker_command.acceleration.y;
    cmd.acceleration.z += tracker_command.acceleration.z;
  }

  if (tracker_command.use_heading_rate) {
    cmd.heading_rate += tracker_command.heading_rate;
  }

  // | ----------------- set the control output ----------------- |

  // the following types can be set as control_output
  // ------------------------------------------------------
  // mrs_msgs::msg::HwApiPositionCmd
  // mrs_msgs::msg::HwApiVelocityHdgCmd
  // mrs_msgs::msg::HwApiVelocityHdgRateCmd
  // mrs_msgs::msg::HwApiAccelerationHdgCmd
  // mrs_msgs::msg::HwApiAccelerationHdgRateCmd
  // mrs_msgs::msg::HwApiAttitudeCmd
  // mrs_msgs::msg::HwApiAttitudeRateCmd
  // mrs_msgs::msg::HwApiControlGroupCmd
  // mrs_msgs::msg::HwApiActuatorCmd
  // ------------------------------------------------------
  // - the controller can check the "capabilities" of the underlying
  //   hardware in common_handlers_->control_output_modalities
  // - the controller can output any of the available modalities based
  //   on your choice
  // - the modalities that your controller is aable to output should be
  //   listed in it's configuration in the custom_config
  // - if the controller's modalities listed in custom_config do have
  //   zero overlap with the HW API's offered modalities, the controller
  //   won't be loaded
  // ------------------------------------------------------
  last_control_output_.control_output = cmd;

  // --------------------------------------------------------------
  // |                 fill in the optional parts                 |
  // --------------------------------------------------------------

  // it is recommended to fill the optional parts if you know them

  // | ------------------- desired orientation ------------------ |

  /// the "desired_orientation" is used for:
  // * plotting the orientation in the control_manager/control_refence topic
  // * checking for attitude control error, which can trigger eland/failsafe
  // last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(...);

  // | -------------- unbiased desired acceleration ------------- |

  /// IMPORANT
  // The acceleration and heading rate in 3D (expressed in the "fcu" frame of reference)
  // that the UAV will actually undergo due to the control action.
  Eigen::Vector3d unbiased_des_acc(0, 0, 0);

  {
    Eigen::Vector3d unbiased_des_acc_world(cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z);

    geometry_msgs::msg::Vector3Stamped world_accel;

    world_accel.header.stamp    = clock_->now();
    world_accel.header.frame_id = uav_state.header.frame_id;
    world_accel.vector.x        = unbiased_des_acc_world(0);
    world_accel.vector.y        = unbiased_des_acc_world(1);
    world_accel.vector.z        = unbiased_des_acc_world(2);

    auto res = common_handlers_->transformer->transformSingle(world_accel, "fcu");

    if (res) {
      unbiased_des_acc << res.value().vector.x, res.value().vector.y, res.value().vector.z;
    }
  }

  // fill the unbiased desired acceleration
  last_control_output_.desired_unbiased_acceleration = unbiased_des_acc;

  // | ------------------ desired heading rate ------------------ |

  // the desired heading rate (if known) will be used to calculated feedforward
  // in the form of desired intrinsic yaw rate
  last_control_output_.desired_heading_rate = cmd.heading_rate;

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.controller = "ExampleController";

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::msg::ControllerStatus ExampleController::getStatus() {

  mrs_msgs::msg::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void ExampleController::switchOdometrySource([[maybe_unused]] const mrs_msgs::msg::UavState& new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void ExampleController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response> ExampleController::setConstraints(
    [[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request>& constraints) {

  if (!is_initialized_) {
    return std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>(new mrs_msgs::srv::DynamicsConstraintsSrv::Response());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  RCLCPP_INFO(node_->get_logger(), "updating constraints");

  mrs_msgs::srv::DynamicsConstraintsSrv::Response res;
  res.success = true;
  res.message = "constraints updated";

  return std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>(new mrs_msgs::srv::DynamicsConstraintsSrv::Response(res));
}

//}

}  // namespace example_controller_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(example_controller_plugin::ExampleController, mrs_uav_managers::Controller)
