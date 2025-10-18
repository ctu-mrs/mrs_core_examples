cmake_minimum_required(VERSION 3.5)
project(mrs_core_examples)

set(DEPENDENCIES
  ament_cmake
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_package()
