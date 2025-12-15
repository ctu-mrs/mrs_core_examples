cmake_minimum_required(VERSION 3.28)
project(example_controller_plugin)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[example_controller_plugin]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra" "-Wpedantic")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

# Find all dependencies of this package
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(mrs_uav_managers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(mrs_uav_testing REQUIRED)

## --------------------------------------------------------------
## |                  Example Controller Plugin                 |
## --------------------------------------------------------------

# Define the shared library created by this package
add_library(ExampleControllerPlugin_ExampleController SHARED)

# Add sources to the shared library
# - Source files should be listed in private section.
# - Header files should be added to a file set.
#   - In this case, the headers are not meant to be installed, so we create
#     a private file set. (If you need to install the headers, see the
#     `example_plugin_manager` which shows how to specify public headers)
target_sources(ExampleControllerPlugin_ExampleController
  PRIVATE
    "src/example_controller.cpp"
  PRIVATE FILE_SET private_headers TYPE HEADERS BASE_DIRS "include"
  FILES
    "include/pid.hpp"
)

# Link to the dependencies used by the library
# This call automatically sets up include paths as well as linking.
# Prefer using the `<package>::<lib>` syntax as it provides better error
# handling in CMake. Most packages export targets in the format
# `<package>::<package>` that include the whole library.
# Message libraries unfortunately do not provide this target and users must link
# against: `${<package>_TARGETS}`.
target_link_libraries(ExampleControllerPlugin_ExampleController
  PRIVATE
    rclcpp::rclcpp
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    mrs_uav_managers::mrs_uav_managers
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
)

target_compile_definitions(ExampleControllerPlugin_ExampleController PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

endif()

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ExampleControllerPlugin_ExampleController
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

ament_package()
