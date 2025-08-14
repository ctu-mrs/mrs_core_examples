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
// #include <example_controller_plugin/example_controllerConfig.h>

//}


namespace example_controller_plugin
{

struct DrsParams
{
  double roll;
  double pitch;
  double yaw;
  double force;
};


namespace example_controller
{

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

  const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>  setConstraints(const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;


  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::msg::UavState uav_state_;
  std::mutex         mutex_uav_state_;


  // | --------------- dynamic reconfigure server --------------- |
  std::shared_ptr<mrs_lib::DynparamMgr>       dynparam_mgr_;
  std::mutex                     mutex_dynamic_reconfigure_;
  DrsParams drs_params_;
  void callbackDrs(const std::string param_name, int level);

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::msg::DynamicsConstraints constraints_;
  std::mutex                   mutex_constraints_;

  // | --------- throttle generation and mass estimation -------- |

  double _uav_mass_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  rclcpp::Time        last_update_time_;
  std::atomic<bool> first_iteration_ = true;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

bool ExampleController::initialize(const rclcpp::Node::SharedPtr& node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                   std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers){
    
    node_ = node;
    clock_ = node_->get_clock();

    common_handlers_  = common_handlers;
    private_handlers_ = private_handlers;

    _uav_mass_ = common_handlers->getMass();

    last_update_time_ = clock_->now();

    bool success = true;
    
    // TODO: callback function for drs
    dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_dynamic_reconfigure_);

    success &= private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("controller_plugin") + "/config/example_controller.yaml");

    dynparam_mgr_->get_param_provider().copyYamls(private_handlers->param_loader->getParamProvider());

    dynparam_mgr_->register_param("desired_roll",&drs_params_.roll);
    dynparam_mgr_->register_param("desired_pitch",&drs_params_.pitch);
    dynparam_mgr_->register_param("desired_yaw",&drs_params_.yaw);
    dynparam_mgr_->register_param("desired_thrust_force",&drs_params_.force);

    if (!success) {
        return false;
    }

    private_handlers->param_loader->loadParam("desired_roll", drs_params_.roll);
    private_handlers->param_loader->loadParam("desired_pitch", drs_params_.pitch);
    private_handlers->param_loader->loadParam("desired_yaw", drs_params_.yaw);
    private_handlers->param_loader->loadParam("desired_thrust_force", drs_params_.force);

    // | ------------------ finish loading params ----------------- |

    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "[ExampleController]: could not load all parameters!");
        return false;
    }

    // | ----------------------- finish init ---------------------- |

    RCLCPP_INFO(node_->get_logger(), "[ExampleController]: initialized");

    is_initialized_ = true;

    return true;

}

/* //{ activate() */

bool ExampleController::activate(const ControlOutput& last_control_output) {

  activation_control_output_ = last_control_output;

  first_iteration_ = true;

  is_active_ = true;

  RCLCPP_INFO(node_->get_logger(),"[ExampleController]: activated");

  return true;
}

//}

/* //{ deactivate() */

void ExampleController::deactivate(void) {

  is_active_       = false;
  first_iteration_ = false;

  RCLCPP_INFO(node_->get_logger(), "[ExampleController]: deactivated");
}

//}

/* //{ deactivate() */

void ExampleController::destroy(void) {

  is_active_       = false;
  first_iteration_ = false;

  RCLCPP_INFO(node_->get_logger(), "[ExampleController]: destroyed");
}

//}

/* updateInactive() //{ */

void ExampleController::updateInactive(const mrs_msgs::msg::UavState &uav_state, [[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand>& tracker_command) {

  mrs_lib::set_mutexed(mutex_dynamic_reconfigure_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateActive() */

ExampleController::ControlOutput ExampleController::updateActive(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand& tracker_command) {

    auto drs_params = mrs_lib::get_mutexed(mutex_dynamic_reconfigure_, drs_params_);

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

        RCLCPP_DEBUG(node_->get_logger(), "[ExampleController]: the last odometry message came too close (%.2f s)!", dt);
        dt = 0.01;
    }

    // | -------- check for the available output modalities ------- |

    // you can decide what to return, but it needs to be available
    if (common_handlers_->control_output_modalities.attitude) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1.0, "[ExampleController]: desired attitude output modality is available");
    }

    // | ---------- extract the detailed model parameters --------- |

    if (common_handlers_->detailed_model_params) {

        mrs_uav_managers::control_manager::DetailedModelParams_t detailed_model_params = common_handlers_->detailed_model_params.value();

        RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1.0, "[ExampleController]: UAV inertia is: " << detailed_model_params.inertia);
    }

    // | -------------- prepare the control reference ------------- |

    geometry_msgs::msg::PoseStamped position_reference;

    position_reference.header           = tracker_command.header;
    position_reference.pose.position    = tracker_command.position;
    position_reference.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(tracker_command.heading);

    // | ---------------- prepare the control output --------------- |

    mrs_msgs::msg::HwApiAttitudeCmd attitude_cmd;

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

const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>  ExampleController::setConstraints([
    [maybe_unused]] const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request>& constraints) {

  if (!is_initialized_) {
    return std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>(new mrs_msgs::srv::DynamicsConstraintsSrv::Response());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  RCLCPP_INFO(node_->get_logger(), "[ExampleController]: updating constraints");

  mrs_msgs::srv::DynamicsConstraintsSrv::Response res;
  res.success = true;
  res.message = "constraints updated";

  return std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>(new mrs_msgs::srv::DynamicsConstraintsSrv::Response(res));
  
}

//}


// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void ExampleController::callbackDrs(const std::string param_name,[[maybe_unused]] int level) {

  // In ros2 updating of the parameters is handled by DynparamMgr class.
  // callback function to be used for logging and triggering action with dynamic parameter change.

  RCLCPP_INFO(node_->get_logger(),"[ExampleController]: dynamic reconfigure params updated" ,param_name.c_str());
}

//}

} // namespace example_controller

} // namespace example_controller_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(example_controller_plugin::example_controller::ExampleController, mrs_uav_managers::Controller)


