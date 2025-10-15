#pragma once

#include <eigen3/Eigen/Eigen>
#include <mrs_lib/param_loader.h>

namespace example_plugin_manager
{

struct PrivateHandlers_t
{
  std::unique_ptr<mrs_lib::ParamLoader> param_loader;
  std::shared_ptr<mrs_lib::ParamLoader> parent_param_loader;
  std::string                           name_space;
  std::string                           runtime_name;
};

}  // namespace example_plugin_manager
