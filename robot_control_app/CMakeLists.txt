cmake_minimum_required(VERSION 3.0.2)
project(robot_control_app)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  my_robot_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

catkin_package(
  CATKIN_DEPENDS roscpp actionlib my_robot_msgs sensor_msgs image_transport cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(robot_control_app src/robot_control_app.cpp)
target_link_libraries(robot_control_app ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Qt5::Widgets)
add_dependencies(robot_control_app ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
