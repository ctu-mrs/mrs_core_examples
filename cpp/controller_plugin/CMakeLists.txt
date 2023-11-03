cmake_minimum_required(VERSION 3.1.2)
project(example_controller_plugin)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  rospy
  pluginlib
  geometry_msgs
  dynamic_reconfigure
  mrs_uav_managers
  mrs_msgs
  mrs_lib
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

generate_dynamic_reconfigure_options(
  cfg/example_controller.cfg
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

set(LIBRARIES
  ExampleController
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# Example Controller

add_library(ExampleController
  src/example_controller.cpp
  )

add_dependencies(ExampleController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(ExampleController
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
