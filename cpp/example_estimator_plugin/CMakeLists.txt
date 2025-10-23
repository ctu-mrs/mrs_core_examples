cmake_minimum_required(VERSION 3.8)
project(example_estimator_plugin)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[example_estimator_plugin]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  mrs_lib
  mrs_msgs
  pluginlib
  mrs_uav_managers
  mrs_uav_state_estimators
  mrs_uav_testing
  Eigen3
)

# define our exported libraries and put them into a list
# ... the list is going ot be reused later multiple times
set(LIBRARIES
  ExampleEstimatorPlugin_ExampleEstimator
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${mrs_lib_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_uav_managers_INCLUDE_DIRS}
  ${mrs_uav_state_estimators_INCLUDE_DIRS}
  ${mrs_uav_testing_INCLUDE_DIRS}
)

# |------------------------------------------------------|
# |             Example estimator plugin                 |
# |------------------------------------------------------|

add_library(ExampleEstimatorPlugin_ExampleEstimator SHARED
  src/example_estimator_plugin.cpp
)

ament_target_dependencies(ExampleEstimatorPlugin_ExampleEstimator
  ${DEPENDENCIES}
)

target_compile_definitions(ExampleEstimatorPlugin_ExampleEstimator PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
