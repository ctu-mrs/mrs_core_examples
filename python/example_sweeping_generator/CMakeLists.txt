cmake_minimum_required(VERSION 3.8)
project(example_sweeping_generator)

set(DEPENDENCIES
  ament_cmake
  ament_cmake_python
  rclpy
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY ${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  DESTINATION lib
  )

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
  )

ament_python_install_package(${PROJECT_NAME})

ament_package()
