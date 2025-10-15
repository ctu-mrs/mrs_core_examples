#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/state_estimator.h>
#include <mrs_uav_state_estimators/estimators/state/state_generic.h>

namespace example_estimator_plugin
{

const char estimator_name[] = "example_estimator_plugin";
const bool is_core_plugin   = false;

class ExampleEstimator : public mrs_uav_state_estimators::StateGeneric {
public:
  ExampleEstimator() : StateGeneric(estimator_name, is_core_plugin) {
  }
};

}  // namespace example_estimator_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(example_estimator_plugin::ExampleEstimator, mrs_uav_managers::StateEstimator)
