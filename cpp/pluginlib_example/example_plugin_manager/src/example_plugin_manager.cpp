#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/node.h>

#include <pluginlib/class_loader.hpp>
#include <example_plugin_manager/plugin_interface.h>

namespace example_plugin_manager
{

/* PluginParams //{ */

class PluginParams {

public:
  PluginParams(const std::string& address, const std::string& name_space, const double& some_property);

  std::string address;
  std::string name_space;
  double      some_property;
};

PluginParams::PluginParams(const std::string& address, const std::string& name_space, const double& some_property) {

  this->address       = address;
  this->name_space    = name_space;
  this->some_property = some_property;
}

//}

/* class ExamplePluginManager //{ */

class ExamplePluginManager : public mrs_lib::Node {

public:
  ExamplePluginManager(const rclcpp::NodeOptions& options);

  // should the initialise method be virtual ? //
  void initialize();

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
  bool                     is_initialized_ = false;

  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;

  // | ---------------------- update timer ---------------------- |

  rclcpp::TimerBase::SharedPtr timer_update_;
  double                       _rate_timer_update_;

  // | -------- an object we want to share to our plugins ------- |

  std::string example_of_a_shared_object_;

  // | --------------------- common handlers -------------------- |

  std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers_;

  // | --------------- dynamic loading of plugins --------------- |

  std::unique_ptr<pluginlib::ClassLoader<example_plugin_manager::Plugin>> plugin_loader_;  // pluginlib loader
  std::vector<std::string>                                                _plugin_names_;
  std::map<std::string, PluginParams>                                     plugins_;      // map between plugin names and plugin params
  std::vector<std::shared_ptr<example_plugin_manager::Plugin>>            plugin_list_;  // list of plugins, routines are callable from this
  std::mutex                                                              mutex_plugins_;

  std::string _initial_plugin_name_;
  int         _initial_plugin_idx_ = 0;

  int active_plugin_idx_ = 0;

  // | ------------------------ routines ------------------------ |

  double vectorNorm(const Eigen::Vector3d& input);

  // | ------------------------- timers ------------------------- |

  void timerUpdate();
};

//}

/* ExamplePluginManager() //{ */

ExamplePluginManager::ExamplePluginManager(const rclcpp::NodeOptions& options) : Node("example_plugin_manager", options) {

  this->initialize();
}

//}

/* initialize() //{ */

void ExamplePluginManager::initialize() {

  node_ = this->this_node_ptr();

  clock_ = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "initializing");

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  param_loader_ = std::make_shared<mrs_lib::ParamLoader>(node_, "ExamplePluginManager");

  param_loader_->addYamlFileFromParam("config");
  param_loader_->addYamlFileFromParam("plugin_config");

  param_loader_->loadParam("update_timer_rate", _rate_timer_update_);
  param_loader_->loadParam("initial_plugin", _initial_plugin_name_);

  // | --------------- example of a shared object --------------- |

  example_of_a_shared_object_ = "Hello, this is a shared object";

  // --------------------------------------------------------------
  // |                       common handlers                      |
  // --------------------------------------------------------------

  common_handlers_ = std::make_shared<example_plugin_manager::CommonHandlers_t>();

  common_handlers_->some_shared_object = std::make_shared<std::string>(example_of_a_shared_object_);

  common_handlers_->vector_calculator.vectorNorm = std::bind(&ExamplePluginManager::vectorNorm, this, std::placeholders::_1);
  common_handlers_->vector_calculator.enabled    = true;

  // --------------------------------------------------------------
  // |                      load the plugins                      |
  // --------------------------------------------------------------

  param_loader_->loadParam("plugins", _plugin_names_);

  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<example_plugin_manager::Plugin>>("example_plugin_manager", "example_plugin_manager::Plugin");

  // for each plugin in the list
  for (int i = 0; i < int(_plugin_names_.size()); i++) {

    std::string plugin_name = _plugin_names_[i];

    // load the plugin parameters
    std::string address;
    std::string name_space;
    double      some_property;

    param_loader_->loadParam(plugin_name + "/address", address);
    param_loader_->loadParam(plugin_name + "/name_space", name_space);
    param_loader_->loadParam(plugin_name + "/some_property", some_property);

    PluginParams new_plugin(address, name_space, some_property);
    plugins_.insert(std::pair<std::string, PluginParams>(plugin_name, new_plugin));

    try {
      RCLCPP_INFO(node_->get_logger(), "loading the plugin '%s'", new_plugin.address.c_str());
      plugin_list_.push_back(plugin_loader_->createSharedInstance(new_plugin.address.c_str()));
    }
    catch (pluginlib::CreateClassException& ex1) {
      RCLCPP_ERROR(node_->get_logger(), "CreateClassException for the plugin '%s'", new_plugin.address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex1.what());
      rclcpp::shutdown();
    }
    catch (pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "PluginlibException for the plugin '%s'", new_plugin.address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex.what());
      rclcpp::shutdown();
    }
  }

  RCLCPP_INFO(node_->get_logger(), "plugins were loaded");

  rclcpp::Node::SharedPtr plugins_subnode_ = node_->create_sub_node("common_plugin_namespace");

  for (int i = 0; i < int(plugin_list_.size()); i++) {
    try {

      std::map<std::string, PluginParams>::iterator it;

      it = plugins_.find(_plugin_names_[i]);

      RCLCPP_INFO(node_->get_logger(), "initializing the plugin '%s'", it->second.address.c_str());

      rclcpp::Node::SharedPtr subnode = plugins_subnode_->create_sub_node(it->second.name_space);

      std::shared_ptr<example_plugin_manager::PrivateHandlers_t> private_handlers = std::make_shared<example_plugin_manager::PrivateHandlers_t>();

      private_handlers->name_space   = it->second.name_space;
      private_handlers->runtime_name = _plugin_names_.at(i);
      private_handlers->param_loader = std::make_unique<mrs_lib::ParamLoader>(subnode, _plugin_names_.at(i));
      private_handlers->param_loader->copyYamls(*param_loader_);
      private_handlers->parent_param_loader = std::make_unique<mrs_lib::ParamLoader>(node_, _plugin_names_.at(i));
      private_handlers->parent_param_loader->copyYamls(*param_loader_);

      plugin_list_[i]->initialize(subnode, common_handlers_, private_handlers);
    }

    catch (std::runtime_error& ex) {
      RCLCPP_ERROR(node_->get_logger(), "exception caught during plugin initialization: '%s'", ex.what());
    }
  }

  RCLCPP_INFO(node_->get_logger(), "plugins were initialized");

  // --------------------------------------------------------------
  // |          check for existance of the initial plugin         |
  // --------------------------------------------------------------

  {
    bool check = false;

    for (int i = 0; i < int(_plugin_names_.size()); i++) {

      std::string plugin_name = _plugin_names_[i];

      if (plugin_name == _initial_plugin_name_) {
        check                = true;
        _initial_plugin_idx_ = i;
        break;
      }
    }
    if (!check) {
      RCLCPP_ERROR(node_->get_logger(), "the initial plugin (%s) is not within the loaded plugins", _initial_plugin_name_.c_str());
      rclcpp::shutdown();
    }
  }

  // | ---------- activate the first plugin on the list --------- |

  RCLCPP_INFO(node_->get_logger(), "activating plugin with idx %d on the list (named: %s)", _initial_plugin_idx_, _plugin_names_[_initial_plugin_idx_].c_str());

  int some_activation_input_to_plugin = 1234;

  plugin_list_[_initial_plugin_idx_]->activate(some_activation_input_to_plugin);
  active_plugin_idx_ = _initial_plugin_idx_;

  // | ------------------------- timers ------------------------- |

  timer_update_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / _rate_timer_update_), std::bind(&ExamplePluginManager::timerUpdate, this));

  // | ----------------------- finish init ---------------------- |

  if (!param_loader_->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    rclcpp::shutdown();
  }

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

// | ------------------------- timers ------------------------- |

/* timerUpdate() //{ */

void ExamplePluginManager::timerUpdate() {

  if (!is_initialized_) {
    return;
  }

  auto active_plugin_idx = mrs_lib::get_mutexed(mutex_plugins_, active_plugin_idx_);

  // input for the plugin
  Eigen::Vector3d input;
  input << 0, 1, 2;

  // call the plugin's update routine
  auto result = plugin_list_[active_plugin_idx]->update(input);

  if (result) {

    // print the result
    RCLCPP_INFO(node_->get_logger(), "plugin update() returned: %.2f", result.value());

  } else {

    RCLCPP_ERROR(node_->get_logger(), "plugin update failed!");
  }
}

//}

// | ------------------------ routines ------------------------ |

/* vectorNorm() //{ */

double ExamplePluginManager::vectorNorm(const Eigen::Vector3d& input) {

  RCLCPP_INFO(node_->get_logger(), "somebody called the manager's vectorNorm() function, probably some plugin");

  return input.norm();
}

//}

}  // namespace example_plugin_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(example_plugin_manager::ExamplePluginManager)
