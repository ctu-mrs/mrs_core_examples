#include <rclcpp/rclcpp.hpp>
#include <rclcpp/package.hpp>

#include <example_plugin_manager/plugin_interface.h>

#include <mrs_lib/param_loader.hpp>


namespace example_plugins
{

namespace example_plugin
{

/* class ExamplePlugin //{ */

class ExamplePlugin : public example_plugin_manager::Plugin {

public:
  void initialize(std::shared_ptr<rclcpp::Node> parent_node_, const std::string& name, const std::string& name_space,
                  std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers);

  bool activate(const int& some_number);
  void deactivate(void);

  const std::optional<double> update(const Eigen::Vector3d& input);

  // parameter from a config file
  double _pi_;

  std::string _name_;

private:
  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers_;
};

//}

// | -------------------- plugin interface -------------------- |

/* initialize() //{ */

void ExamplePlugin::initialize(std::shared_ptr<rclcpp::Node> parent_node_, const std::string& name, const std::string& name_space,
                               std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers) {

  //node_ will behave just like normal Nodehandle
  node_ = parent_node_;

  _name_ = name;

  // I can use this to get stuff from the manager interactively
  common_handlers_ = common_handlers;

  // | ------------------- loading parameters ------------------- |

  param_loader =  mr_lib::param_loader(node_, "ExamplePlugin");

  param_loader.addYamlfromparam('config');

  // can load params like in a ROS node
  param_loader.loadParam("pi", _pi_);

  if (!load_successfully) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: could not load all parameters!", _name_.c_str());
    rclcpp::shutdown();
  }

  RCLCPP_INFO(node_->get_logger(), "[%s]: loaded custom parameter: pi=%f", _name_.c_str(), _pi_);

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: initialized under the name '%s', and namespace '%s'", _name_.c_str(), name.c_str(), name_space.c_str());

  is_initialized_ = true;
}

//}

/* activate() //{ */

bool ExamplePlugin::activate(const int& some_number) {

  RCLCPP_INFO(node_->get_logger(), "[%s]: activated with some_number=%d", _name_.c_str(), some_number);

  is_active_ = true;

  return true;
}

//}

/* deactivate() //{ */

void ExamplePlugin::deactivate(void) {

  is_active_ = false;

  RCLCPP_INFO(node_->get_logger(), "[%s]: deactivated", _name_.c_str());
}

//}



/* update() //{ */

const std::optional<double> ExamplePlugin::update(const Eigen::Vector3d& input) {

  if (!is_active_) {
    return false;
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "[" << _name_ << "]: update() was called, let's find out the size of the vector [" << input.transpose() << "]");

  // check some property from the "manager"
  if (common_handlers_->vector_calculator.enabled) {

    // use a function from the common_handlers
    double vector_norm = common_handlers_->vector_calculator.vectorNorm(input);

    // we calculated our result, just return it to the manager
    return vector_norm;

  } else {

    return false;
  
  }

}

//}

}  // namespace example_plugin
}  // namespace example_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(example_plugins::example_plugin::ExamplePlugin, example_plugin_manager::Plugin)