cmake_minimum_required(VERSION 3.0.2)
project(reverse_manoeuvre_pkg)

## Find catkin and any dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

## Declare a catkin package
catkin_package()

## Include headers
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Build the executable
add_executable(reverse_node 
    src/main.cpp 
    src/reverse_manoeuvre.cpp
)

## Link libraries
target_link_libraries(reverse_node
  ${catkin_LIBRARIES}
)