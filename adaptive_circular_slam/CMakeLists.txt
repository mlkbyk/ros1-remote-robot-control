cmake_minimum_required(VERSION 3.0.2)
project(adaptive_circular_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)

#

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   std_msgs
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES adaptive_circular_slam
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(circle_generator_node src/circle_generator.cpp)
target_link_libraries(circle_generator_node ${catkin_LIBRARIES})

add_executable(circle_scoring_node src/circle_scoring.cpp)
target_link_libraries(circle_scoring_node ${catkin_LIBRARIES})

add_executable(motion_executor_node src/motion_executor.cpp)
target_link_libraries(motion_executor_node ${catkin_LIBRARIES})



