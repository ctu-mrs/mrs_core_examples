#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/lkf.h>
#include <mrs_lib/repredictor.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/dynparam_mgr.h>


#include <mrs_uav_state_estimators/estimators/state/state_generic.h>


namespace example_estimator_plugin
{

const char estimator_name[] = "example_estimator_plugin";
const bool is_core_plugin   = false;

/* package name is defined as a namespace property with default value of 'mrs_uav_state_estimators'. We have to override it.*/
namespace state_generic
{
const char package_name[] = "example_estimator_plugin";
}

class ExampleEstimator : mrs_uav_state_estimators::StateGeneric
{
public:
    ExampleEstimator():StateGeneric(estimator_name, is_core_plugin){}

    // | --------------- param loader description ----------------- |
    /* When plugin is not core plugin, public and private paramters are loaded and therefore they need to be specified in the custom package.*/
    /* How does it work with hardware though ?: Only one type of hardware can be used for whole flight. hardware is not changed midflight. */
    ph->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/private/" + getName() + ".yaml");
    ph->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/public/" + getName() + ".yaml");
  

    // | --------------- subscriber initialization ---------------- |

    

    // | ---------------- publishers initialization --------------- |

    // ph_odom_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(this, "~/" + Support::toSnakeCase(getName()) + "/odom");





};
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(example_estimator_plugin::ExampleEstimator, mrs_uav_managers::StateEstimator)