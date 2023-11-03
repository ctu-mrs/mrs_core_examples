cmake_minimum_required(VERSION 3.15.0)
project(example_waypoint_flier)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CATKIN_DEPENDENCIES
  roscpp
  nodelet
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  )

SET(LIBRARIES
  WaypointFlier
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

## Generate dynamic parameters which are loaded by the dynamic reconfigure server
generate_dynamic_reconfigure_options(
  config/dynparam.cfg
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS Eigen
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(WaypointFlier
  src/waypoint_flier.cpp
  )

## Add configure headers for dynamic reconfigure
add_dependencies(WaypointFlier
  ${PROJECT_NAME}_gencfg
  )

## Specify libraries to link a library or executable target against
target_link_libraries(WaypointFlier
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
