cmake_minimum_required(VERSION 3.15.0)
project(example_sweeping_generator)

set(DEPENDENCIES
  ament_cmake
  ament_cmake_python
  rclpy
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

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
  FILES ${PROJECT_NAME}/sweeping_generator.py
  DESTINATION lib/${PROJECT_NAME}
  )

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
