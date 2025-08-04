cmake_minimum_required(VERSION 3.8)
project(example_plugin_manager)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(DEPENDENCIES
  ament_cmake
  pluginlib
  rclcpp
  mrs_lib
  rclcpp_components
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  example_plugin_manager
  )

# exmple_plugin_manager
add_library(example_plugin_manager SHARED src/example_plugin_manager.cpp)

ament_target_dependencies(example_plugin_manager
  mrs_lib
  pluginlib
  rclcpp
  rclcpp_components
)

target_include_directories(example_plugin_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mrs_lib_INCLUDE_DIRS}
)

# each component (nodelet) needs to be registered to make it discoverable at runtime
rclcpp_components_register_nodes(example_plugin_manager "example_plugin_manager::ExamplePluginManager" EXECUTABLE ExamplePluginManager)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_targets(
  export_${PROJECT_NAME} 
)

ament_export_include_directories(
  include
)

ament_package()
