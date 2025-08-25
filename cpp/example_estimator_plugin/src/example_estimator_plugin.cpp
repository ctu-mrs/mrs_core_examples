#include <rclcpp/rclcpp.hpp>

#include <example_plugin_manager/plugin_interface.h>

#include <mrs_lib/param_loader.h>

#include <mrs_uav_state_estimators/estimators/state/state_generic.h>


namespace example_estimator_plugin
{

const char estimator_name[] = "example_estimator_plugin";
const bool is_core_plugin   = false;

class ExampleEstimator : mrs_uav_state_estimators::StateGeneric
{
public:
    ExampleEstimator():StateGeneric(estimator_name, is_core_plugin){
        
    }
    

private:


};
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(example_estimator_plugin::ExampleEstimator, mrs_uav_managers::StateEstimator)