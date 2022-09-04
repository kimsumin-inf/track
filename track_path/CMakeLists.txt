cmake_minimum_required(VERSION 3.0.2)
project(track_path)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
        novatel_gps_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(INCLUDE_DIRS include
CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs roscpp std_msgs tf)


include_directories(include
  ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/track_path_node.cpp include/track_path/track_path.h src/track_path.cpp include/track_path/navsat_conversions.h)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${OpenCV_LIBS})
