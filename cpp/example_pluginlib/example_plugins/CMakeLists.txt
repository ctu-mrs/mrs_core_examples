cmake_minimum_required(VERSION 3.8)
project(example_plugins)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# find dependencies
set(DEPENDENCIES
  rclcpp
  rclcpp_components
  pluginlib
  mrs_lib
  example_plugin_manager
  mrs_uav_testing
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# define our exported libraries and put them into a list
# ... the list is going ot be reused later multiple times
set(LIBRARIES
  ExamplePlugins_ExamplePlugin
  )

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${example_plugin_manager_INCLUDE_DIRS}
  ${mrs_uav_testing_INCLUDE_DIRS}
  )

## --------------------------------------------------------------
## |                           compile                          |
## --------------------------------------------------------------

add_library(ExamplePlugins_ExamplePlugin SHARED
  src/example_plugins.cpp
)

ament_target_dependencies(ExamplePlugins_ExamplePlugin
  ${DEPENDENCIES}
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

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
