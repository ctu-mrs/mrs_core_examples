#include <rclcpp/rclcpp.hpp>

#include <example_plugin_manager/plugin_interface.h>

#include <mrs_lib/param_loader.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace example_plugins
{

namespace example_plugin
{

/* class ExamplePlugin //{ */

class ExamplePlugin : public example_plugin_manager::Plugin {

public:
  void initialize(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<example_plugin_manager::PrivateHandlers_t> private_handlers);

  bool activate(const int& some_number);

  void deactivate(void);

  const std::optional<double> update(const Eigen::Vector3d& input);

  // parameter from a config file
  double      _pi_;
  std::string _custom_param_;
  std::string _global_custom_param_;

  std::string _name_;

private:
  rclcpp::Node::SharedPtr node_;
  bool                    is_initialized_ = false;

  bool is_active_ = false;

  std::shared_ptr<example_plugin_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<example_plugin_manager::PrivateHandlers_t> private_handlers_;
};

//}

// | -------------------- plugin interface -------------------- |

/* initialize() //{ */

void ExamplePlugin::initialize(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers,
                               std::shared_ptr<example_plugin_manager::PrivateHandlers_t> private_handlers) {

  // node_ is same as the parent node and there are the parameters
  node_ = node;

  // I can use this to get stuff from the manager interactively
  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _name_ = private_handlers->runtime_name;

  // | ------------------- loading parameters ------------------- |

  // add yaml config from the package's config folder
  private_handlers->parent_param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("example_plugins") + "/config/example_plugin.yaml");

  // load global param using the parent's loader
  private_handlers_->parent_param_loader->loadParam("pi", _pi_);

  // this parameter is going to be the same for all the instances of the plugin
  private_handlers->parent_param_loader->loadParam("global_custom_param", _global_custom_param_);

  // load custom param from the plugin's namespace, this might be different for each instance of the plugin
  private_handlers->param_loader->loadParam("custom_param", _custom_param_);

  if (!private_handlers_->parent_param_loader->loadedSuccessfully() || !private_handlers_->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: could not load all parameters!", _name_.c_str());
    rclcpp::shutdown();
    exit(1);
  }

  RCLCPP_INFO(node_->get_logger(), "[%s]: loaded parameter: pi=%f", _name_.c_str(), _pi_);
  RCLCPP_INFO(node_->get_logger(), "[%s]: loaded parameter: custom_param=%s", _name_.c_str(), _custom_param_.c_str());
  RCLCPP_INFO(node_->get_logger(), "[%s]: loaded parameter: global_custom_param=%s", _name_.c_str(), _global_custom_param_.c_str());

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: initialized under the name '%s', and namespace '%s'", _name_.c_str(), _name_.c_str(),
              private_handlers->name_space.c_str());

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
