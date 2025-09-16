cmake_minimum_required(VERSION 3.8)
project(example_estimator_plugin)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
    pluginlib
    rclcpp
    mrs_lib
    mrs_msgs
    mrs_uav_managers
    mrs_uav_state_estimators
    )

    
foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
)

# |------------------------------------------------------|
# |             Example estimator plugin                 |
# |------------------------------------------------------|
add_library(ExampleEstimator src/example_estimator_plugin.cpp)

ament_target_dependencies(ExampleEstimator
  mrs_lib
  pluginlib
  rclcpp
  mrs_msgs
  mrs_uav_managers
  mrs_uav_state_estimators
)

target_include_directories(ExampleEstimator PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mrs_lib_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ExampleEstimator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

# ament_export_libraries(
#   ExampleEstimator
# )

# ament_export_targets(
#   export_${PROJECT_NAME}  
# )

ament_package()