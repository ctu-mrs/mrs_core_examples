cmake_minimum_required(VERSION 3.5)
project(example_tracker_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")



set(DEPENDENCIES
  rclcpp
  mrs_lib
  mrs_msgs
  mrs_uav_managers
  Eigen3
  )

foreach(DEP IN LIST DEPENEDENCIES)
  find(${DEP} REQUIRED)
endforeach()



include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${colcon_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )



# set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
# set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

set(LIBRARIES
  ExampleTracker
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
  )

## --------------------------------------------------------------
## |                            Build                           |
## --------------------------------------------------------------


# Example Tracker

add_library(ExampleTracker
  src/example_tracker.cpp
  )

add_dependencies(ExampleTracker
  ${DEPENDENCIES}
  )

target_link_libraries(ExampleTracker
  ${LIBRARIES}
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
