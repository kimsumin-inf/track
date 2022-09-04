//
// Created by sumin on 22. 9. 4.
//

#ifndef SRC_DETECTION_H
#define SRC_DETECTION_H

#include "Filtering.h"
#include "Segmentation.h"

// C++ base Library
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

//pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transformation_from_correspondences.h>

//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

class Detection:public Filtering, Segmentation{
private:
    //subscriber

    ros::Subscriber data_sub;
    ros::Subscriber map_Flag;

    //publisher
    ros::Publisher waypoint_pub;
    ros::Publisher ground_pub;
    ros::Publisher roi_pub;
    ros::Publisher seg_pub;
    ros::Publisher marker_pub;
    ros::Publisher left_pub;
    ros::Publisher right_pub;
    ros::Publisher center_pub;

    //data
    pcl::PointCloud<pcl::PointXYZI> raw_data;
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    bool shutdown;


public:
    Detection(ros::NodeHandle node, ros::NodeHandle pnh);
    void Init();
    void dataCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void map_Flag_CB(const std_msgs::Bool::ConstPtr& msg);
    std_msgs::Bool map_state;
};

#endif //SRC_DETECTION_H
