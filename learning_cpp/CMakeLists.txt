cmake_minimum_required(VERSION 3.0.2)
project(learning_cpp)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  roscpp
  tf
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}

)

# --- NODE 1: Odom Listener ---
add_executable(odom_listener_node src/odom_listener.cpp)

target_link_libraries(odom_listener_node
  ${catkin_LIBRARIES}
)



# --- NODE 2: Lidar Listener---
add_executable(lidar_listener src/lidar_listener.cpp)
target_link_libraries(lidar_listener
  ${catkin_LIBRARIES}
)

# --- NODE 3: Robot Commander ---
add_executable(commander_node src/robot_commander.cpp)
target_link_libraries(commander_node
  ${catkin_LIBRARIES}
)
