cmake_minimum_required(VERSION 3.28)
project(example_plugin_manager)

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
find_package(ament_cmake REQUIRED)

# Dependencies in this list will be available both in this project and any
# project that includes (find_library(...)) this project.
set(DEPENDENCIES
  rclcpp
  rclcpp_components
  pluginlib
  mrs_lib
  Eigen3
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# Define public library for this package
# - Since the whole implementation is in headers only, the library is defined as
#   `INTERFACE`.
# - If there were source files implementing behavior of the headers, `INTERFACE`
#   should be replaced by either STATIC or SHARED (or removed completely,
#   resulting in default type based on BUILD_SHARED_LIBS variable).
# - The name here should only be used when configuring the library. When linking
#   to it, use the alias defined below.
add_library(ExamplePluginManager_ExamplePluginManager INTERFACE)
# Create an alias to unify usage outside and inside this project.
# The convention is that projects export libraries with name
# `<package>::<package>` if there is only one, or
# `<package>::<lib_name>` if there are multiple.
add_library(example_plugin_manager::example_plugin_manager ALIAS ExamplePluginManager_ExamplePluginManager)
# Configure this libraries properties
# - EXPORT_NAME - should be the same as the lib name defined in the alias
#                 (excluding the namespace)
# - OUTPUT_NAME - base name of the binary file created, usually same as export
#                 name
set_target_properties(ExamplePluginManager_ExamplePluginManager
  PROPERTIES
    EXPORT_NAME example_plugin_manager
    OUTPUT_NAME example_plugin_manager
)


# Add sources to the shared library
# - If present, source files should be included in a PRIVATE section similarly
#   to the second defined library below.
# - Header files should be added to a file set.
#   - Since this library is INTERFACE, the file set must also be declared
#     INTERFACE. If we used a STATIC or SHARED library, the file set should be
#     made public to be available both when building this library and linking it
#     in other projects.
target_sources(ExamplePluginManager_ExamplePluginManager
  INTERFACE FILE_SET HEADERS BASE_DIRS "include"
  FILES
    "include/example_plugin_manager/common_handlers.h"
    "include/example_plugin_manager/plugin_interface.h"
    "include/example_plugin_manager/private_handlers.h"
)

# Link to the dependencies used by the library
# This call automatically sets up include paths as well as linking.
# Prefer using the `<package>::<lib>` syntax as it provides better error
# handling in CMake. Most packages export targets in the format
# `<package>::<package>` that include the whole library.
# Message libraries unfortunately do not provide this target and users must link
# against: `${<package>_TARGETS}`.
target_link_libraries(ExamplePluginManager_ExamplePluginManager
  INTERFACE
    rclcpp::rclcpp
    mrs_lib::mrs_lib
    Eigen3::Eigen
)

# Add shared library for the plugin manager component
add_library(ExamplePluginManager_PluginManagerComponent SHARED)

# Add sources to the plugin manager library
target_sources(ExamplePluginManager_PluginManagerComponent
  PRIVATE
    "src/example_plugin_manager.cpp"
)

# Link dependencies
# Linking against the locally defined library is done through the alias to be
# consistent with external uses.
target_link_libraries(ExamplePluginManager_PluginManagerComponent
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    example_plugin_manager::example_plugin_manager
)

# Register components defined in this library.
# rclcpp_components_register_nodes(<library> <names of components>...)
rclcpp_components_register_nodes(ExamplePluginManager_PluginManagerComponent
  "example_plugin_manager::ExamplePluginManager"
)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS
    ExamplePluginManager_ExamplePluginManager
    ExamplePluginManager_PluginManagerComponent
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "install/${PROJECT_NAME}"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
