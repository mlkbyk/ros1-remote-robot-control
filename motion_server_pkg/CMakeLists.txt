cmake_minimum_required(VERSION 3.0.2)
project(motion_server_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  my_robot_msgs
  geometry_msgs
  nav_msgs
  tf2
)

catkin_package(
  CATKIN_DEPENDS roscpp actionlib my_robot_msgs geometry_msgs nav_msgs tf2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motion_server src/motion_server.cpp)
target_link_libraries(motion_server ${catkin_LIBRARIES})
add_dependencies(motion_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
