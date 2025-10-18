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
  FILES ${PROJECT_NAME}/sweeping_generator.py
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
  )

# Install the sources into the .deb package such that they can be
# found even on the installed version of the system.
file(GLOB SOURCE_FILES CONFIGURE_DEPENDS ./*)
list(FILTER SOURCE_FILES EXCLUDE REGEX "\.git.*")
install(
  FILES ${SOURCE_FILES}
  DESTINATION share/${PROJECT_NAME}/source
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
