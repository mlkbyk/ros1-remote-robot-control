cmake_minimum_required(VERSION 3.0.2)
project(motion_client_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  my_robot_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp actionlib my_robot_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motion_client src/motion_client.cpp)
target_link_libraries(motion_client ${catkin_LIBRARIES})
add_dependencies(motion_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(motion_cli src/motion_cli.cpp)
target_link_libraries(motion_cli ${catkin_LIBRARIES})
add_dependencies(motion_cli ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
