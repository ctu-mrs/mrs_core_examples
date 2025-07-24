cmake_minimum_required(VERSION 3.15.0)
project(example_waypoint_flier_simple)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  mrs_msgs
  nav_msgs
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  WaypointFlierSimple
  )

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${colcon_INCLUDE_DIRS}
  )

## --------------------------------------------------------------
## |                     waypointfliersimple                    |
## --------------------------------------------------------------

add_library(WaypointFlierSimple SHARED src/waypoint_flier_simple.cpp
  )

ament_target_dependencies(WaypointFlierSimple
  ${DEPENDENCIES}
  )

# each component (nodelet) needs to be registered to make it discoverable at runtime
rclcpp_components_register_nodes(WaypointFlierSimple PLUGIN "example_waypoint_flier_simple::WaypointFlierSimple" EXECUTABLE WaypointFlierSimple)


## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

ament_export_dependencies(
  ${DEPENDENCIES}
  )

ament_package()
