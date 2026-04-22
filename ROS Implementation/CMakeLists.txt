cmake_minimum_required(VERSION 3.0.2)
project(reverse_manoeuvre_pkg)

## Find catkin and any dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  movai_common
)

## Find yaml-cpp (not a catkin package)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

## Declare a catkin package
catkin_package()

## Include headers
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

set (PACKAGE_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})
set (DOC_DESTINATION /opt/mov.ai/docs/${PROJECT_NAME})

## Build the executable
add_executable(reverse_node 
    src/reverse_manoeuvre_node.cpp 
    src/reverse_manoeuvre.cpp
    src/geometry_solver.cpp
)

## Link libraries
target_link_libraries(reverse_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS reverse_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# 
# install(DIRECTORY ../metadata
#         DESTINATION ${PACKAGE_DESTINATION}
#         )
# 
# install(FILES ../manifest.txt
#         DESTINATION ${PACKAGE_DESTINATION}
# )
# 
# install(FILES ../README.md
#         DESTINATION ${DOC_DESTINATION}
# )