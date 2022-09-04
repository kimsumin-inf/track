//
// Created by sumin on 22. 9. 4.
//

#ifndef SRC_FILTERING_H
#define SRC_FILTERING_H

// C++ base Library
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

// C base Library
#include <ctime>
#include <cmath>

//ros
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/OccupancyGrid.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/convex_hull.h>

//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>


//tf2
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>

class Filtering {
private:
    cv::Point3d min_distance, max_distance;

    double height_thresh, per_cell;
    size_t grid_dim;

public:
    Filtering();
    pcl::PointCloud<pcl::PointXYZI> RoiFilter(pcl::PointCloud<pcl::PointXYZI> before_filter); //passthrough
    pcl::PointCloud<pcl::PointXYZI> GroundFilter(pcl::PointCloud<pcl::PointXYZI> before_filter);
};



#endif //SRC_FILTERING_H
