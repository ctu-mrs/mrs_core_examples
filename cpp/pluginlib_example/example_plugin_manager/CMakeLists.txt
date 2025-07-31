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
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()


# exmple_plugin_manager
add_library(example_plugin_manager src/example_plugin_manager.cpp)

ament_target_dependencies(example_plugin_manager
  mrs_lib
  pluginlib
  rclcpp
)

target_include_directories(example_plugin_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mrs_lib_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(example_plugin_manager plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS example_plugin_manager
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_libraries(
  example_plugin_manager
)

ament_export_targets(
  export_${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
