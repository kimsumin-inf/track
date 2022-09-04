cmake_minimum_required(VERSION 3.0.2)

project(track_lidar)

add_compile_options(-std=c++14)
set(PYTHONPATH /usr/bin/python3)
set(PYTHON_EXECUTABLE /usr/bin/python3)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  data_transfer_msg
  image_geometry
  image_transport
  laser_geometry
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)


catkin_package(INCLUDE_DIRS include
        CATKIN_DEPENDS cv_bridge data_transfer_msg image_geometry image_transport laser_geometry pcl_ros roscpp sensor_msgs std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE project_HEADERS
        "*.h"
        "*.hpp")

add_executable(${PROJECT_NAME} ${project_HEADERS} src/Filtering.cpp src/Segmentation.cpp src/Detection.cpp src/track_lidar_node.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})
