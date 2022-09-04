//
// Created by sumin on 22. 9. 4.
//

#ifndef SRC_SEGMENTATION_H
#define SRC_SEGMENTATION_H

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


struct NODE {
    pcl::PointXYZI point;
//    int count=0;
//    int pre_node=NULL;
//    int next_node=NULL;
//    float angle;
    float slope;
//    int label=0; //LEFT=1, RIGHT=2
};

typedef struct info
{
    int x, y, z;
}info;

class Segmentation{
private:
    int cluster_min, cluster_max;
    double cluster_thresh;
//    pcl::PointCloud<pcl::PointCloud<pcl::PointXYZI>> calc_hull;
    std::vector<pcl::PointXYZI> cen_point;
    std::vector<pcl::PointXYZI> left_point;
    std::vector<pcl::PointXYZI> right_point;
    std::vector<pcl::PointXYZI> way_point;
    pcl::PointXYZI pre_waypoint;

    int direction;
    int class_count=0;
    std::vector<struct NODE> left_graph;
    std::vector<struct NODE> right_graph;

    int left_cen;
    int right_cen;
    float dir_slope=0;
    bool way_value=false;
    float pre_angle, next_angle, final_angle;
    pcl::PointXYZI pre_cen;


    int not_way=0;
    int point=0;
    int dis=0;



public:
    Segmentation();
    pcl::PointCloud<pcl::PointXYZI> Clustering(pcl::PointCloud<pcl::PointXYZI> before_filter, pcl::PointCloud<pcl::PointXYZI> *way_point_filter,pcl::PointCloud<pcl::PointXYZI> *left_filter,pcl::PointCloud<pcl::PointXYZI> *right_filter, pcl::PointCloud<pcl::PointXYZI> *center_filter); //euclidean clustering

    void Make_Direction_Before();
    void Make_Direction_After();

    void Graph_Input(bool left, bool right, NODE& node, float left_slope, float right_slope);
    void Make_Graph(int left, int right);

    void First_Point_Input(int left_or_right, pcl::PointXYZI point, bool& value);
    void First_Point_False(bool& left, bool& right);
    void Classification();

    void Make_Waypoint();


};
#endif //SRC_SEGMENTATION_H
