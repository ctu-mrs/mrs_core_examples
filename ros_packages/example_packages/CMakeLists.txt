cmake_minimum_required(VERSION 2.8.3)
project(example_packages)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
   CATKIN_DEPENDS
)
