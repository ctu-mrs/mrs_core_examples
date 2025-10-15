#pragma once

#include <rclcpp/rclcpp.hpp>
#include <example_plugin_manager/common_handlers.h>
#include <example_plugin_manager/private_handlers.h>

namespace example_plugin_manager
{

class Plugin {
public:
  virtual void initialize(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers,
                          std::shared_ptr<example_plugin_manager::PrivateHandlers_t> private_handlers) = 0;

  virtual bool activate(const int& some_number) = 0;

  virtual void deactivate(void) = 0;

  virtual const std::optional<double> update(const Eigen::Vector3d& input) = 0;

  virtual ~Plugin() = default;
};

}  // namespace example_plugin_manager
