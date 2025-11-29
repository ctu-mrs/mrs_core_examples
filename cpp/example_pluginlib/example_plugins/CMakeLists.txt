cmake_minimum_required(VERSION 3.28)
project(example_plugins)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(example_plugin_manager REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(mrs_uav_testing REQUIRED)

## --------------------------------------------------------------
## |                           compile                          |
## --------------------------------------------------------------

# Define the shared library created by this package
add_library(ExamplePlugins_ExamplePlugin SHARED)

# Add sources to the shared library
# - Source files should be listed in private section.
# - Header files should be added to a file set.
#   - If you are using header files, see other examples on how to properly
#     handle them:
#     - public headers: `example_plugin_manager`
#     - private headers: `example_waypoint_flier_native`
target_sources(ExamplePlugins_ExamplePlugin
  PRIVATE
    "src/example_plugins.cpp"
)

# Link to the dependencies used by the library
# This call automatically sets up include paths as well as linking.
# Prefer using the `<package>::<lib>` syntax as it provides better error
# handling in CMake. Most packages export targets in the format
# `<package>::<package>` that include the whole library.
# Message libraries unfortunately do not provide this target and users must link
# against: `${<package>_TARGETS}` (eg. `${mrs_msgs_TARGETS}`).
target_link_libraries(ExamplePlugins_ExamplePlugin
  PRIVATE
    rclcpp::rclcpp
    pluginlib::pluginlib
    example_plugin_manager::example_plugin_manager
    mrs_lib::mrs_lib
)

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

pluginlib_export_plugin_description_file(example_plugin_manager plugins.xml)

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
  TARGETS ExamplePlugins_ExamplePlugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION "share/${PROJECT_NAME}"
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_package()
