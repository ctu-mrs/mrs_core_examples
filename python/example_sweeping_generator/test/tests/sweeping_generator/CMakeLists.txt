get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(test_${TEST_NAME}
  test.cpp
  )

set(DEPENDENCIES
  rclcpp
  mrs_lib
  mrs_msgs
  mrs_uav_testing
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_uav_testing_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  )

ament_target_dependencies(test_${TEST_NAME}
  ${DEPENDENCIES}
  )

install(TARGETS
  test_${TEST_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )

add_ros_isolated_launch_test(test.py TIMEOUT 300)
