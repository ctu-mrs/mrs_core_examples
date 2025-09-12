#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/lkf.h>
#include <mrs_lib/repredictor.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/dynparam_mgr.h>

#include <mrs_uav_managers/state_estimator.h>
#include <mrs_uav_state_estimators/estimators/state/state_generic.h>


namespace mrs_uav_state_estimators
{

namespace example_estimator_plugin
{

 
const char estimator_name[] = "example_estimator_plugin";
const bool is_core_plugin   = false;

/* package name is defined as a namespace property with default value of 'mrs_uav_state_estimators'. We have to override it.*/
// namespace state_generic
// {
//    char package_name[] = "example_estimator_plugin";
// }

class ExampleEstimator : public mrs_uav_state_estimators::StateGeneric
{
public:
    ExampleEstimator():StateGeneric(estimator_name, is_core_plugin){}

    // | --------------- param loader description ----------------- |
    // ph_->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/private/" + getName() + ".yaml");
    // ph_->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/public/" + getName() + ".yaml");
  

    // | --------------- subscriber initialization ---------------- |
    

    // | ---------------- publishers initialization --------------- |

    // ph_odom_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(this, "~/" + Support::toSnakeCase(getName()) + "/odom");

};
} // namespace example_estimator plugin
} // namespace mrs_uav_state_estimators


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::example_estimator_plugin::ExampleEstimator, mrs_uav_managers::StateEstimator)