//
// Created by sumin on 22. 9. 4.
//

#ifndef SRC_GRAPH_H
#define SRC_GRAPH_H

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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>

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

struct NODE{
    pcl::PointXYZI point;
    int count = 0;
    int pre_node =NULL;
    int next_node =NULL;
    float angle;
    float slope;
    int label = 0; // Left = 1 , Right = 2
};

class Graph{
private :
public:
};

#endif //SRC_GRAPH_H
