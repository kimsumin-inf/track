cmake_minimum_required(VERSION 3.0.2)
project(track_localizer)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        novatel_gps_msgs
        selfcar_lib
        sensor_msgs
        std_msgs
        tf2
        tf2_ros
        )

catkin_package(INCLUDE_DIRS include
        CATKIN_DEPENDS geometry_msgs nav_msgs novatel_gps_msgs selfcar_lib sensor_msgs std_msgs tf2 tf2_ros)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/track_localizer_node.cpp include/track_localizer/track_localizer.h src/track_localizer.cpp src/erp42.cpp src/kalman_filter.cpp
        include/track_localizer/trigonometric.h
        include/track_localizer/covariance.h
        include/track_localizer/erp42.h
        include/track_localizer/kalman_filter.h
        include/track_localizer/navsat_conversions.h
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
