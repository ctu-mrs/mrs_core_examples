cmake_minimum_required(VERSION 3.8)
project(example_plugins)

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
  example_plugin_manager
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()


# exmple_plugins
add_library(ExamplePlugin src/example_plugins.cpp)

ament_target_dependencies(ExamplePlugin
  mrs_lib
  pluginlib
  rclcpp
  example_plugin_manager
)

target_include_directories(ExamplePlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mrs_lib_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(ExamplePlugin plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ExamplePlugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_libraries(
  ExamplePlugin
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
