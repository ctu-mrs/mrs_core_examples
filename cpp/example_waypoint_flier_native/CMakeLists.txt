cmake_minimum_required(VERSION 3.28)
project(example_waypoint_flier_native)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mrs_msgs REQUIRED)

## --------------------------------------------------------------
## |                           compile                          |
## --------------------------------------------------------------

# Define the shared library created by this package
add_library(ExampleWaypointFlierNative_WaypointFlierComponent SHARED)

# Add sources to the shared library
# - Source files should be listed in private section.
# - Header files should be added to a file set.
#   - In this case, the headers are not meant to be installed, so we create
#     a private file set. (If you need to install the headers, see the
#     `example_plugin_manager` which shows how to specify public headers)
target_sources(ExampleWaypointFlierNative_WaypointFlierComponent
  PRIVATE
    "src/waypoint_flier_native.cpp"
  PRIVATE FILE_SET private_headers TYPE HEADERS BASE_DIRS "include"
  FILES
    "include/params.h"
)

# Link to the dependencies used by the library
# This call automatically sets up include paths as well as linking.
# Prefer using the `<package>::<lib>` syntax as it provides better error
# handling in CMake. Most packages export targets in the format
# `<package>::<package>` that include the whole library.
# Message libraries unfortunately do not provide this target and users must link
# against: `${<package>_TARGETS}`.
target_link_libraries(ExampleWaypointFlierNative_WaypointFlierComponent
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    ${nav_msgs_TARGETS}
    ${mrs_msgs_TARGETS}
)

# Register components defined in this library.
# rclcpp_components_register_nodes(<library> <names of components>...)
rclcpp_components_register_nodes(ExampleWaypointFlierNative_WaypointFlierComponent
  "example_waypoint_flier_native::WaypointFlierNative"
)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS ExampleWaypointFlierNative_WaypointFlierComponent
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  DIRECTORY config
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION "share/${PROJECT_NAME}"
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_package()
