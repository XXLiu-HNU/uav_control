cmake_minimum_required(VERSION 3.0.2)
project(my_bspline)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  quadrotor_msgs
  # traj_utils

)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_bspline
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include

  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/bspline.cpp  src/uniform_bspline.cpp)
target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES})
add_executable(fastdrone src/fastdrone.cpp  src/uniform_bspline.cpp)
target_link_libraries(fastdrone ${catkin_LIBRARIES})
