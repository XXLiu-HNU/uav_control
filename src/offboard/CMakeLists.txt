cmake_minimum_required(VERSION 3.0.2)
project(offboard)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  nav_msgs
)


catkin_package(

)

include_directories(
# include
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/offboard_node.cpp)
target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS scripts/vis.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
