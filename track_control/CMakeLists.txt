cmake_minimum_required(VERSION 2.8.3)
project(track_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
        novatel_gps_msgs
  std_msgs
)
catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS ackermann_msgs geometry_msgs nav_msgs roscpp std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/track_control_node.cpp include/track_control/track_control.h src/track_control.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
