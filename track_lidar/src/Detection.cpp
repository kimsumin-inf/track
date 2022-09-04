//
// Created by sumin on 22. 9. 4.
//
#include"track_lidar/Detection.h"

Detection::Detection(ros::NodeHandle node, ros::NodeHandle pnh):Filtering(),Segmentation()
{
    // publish
    waypoint_pub= node.advertise<sensor_msgs::PointCloud2>("waypoint", 1);
    roi_pub=node.advertise<sensor_msgs::PointCloud2>("roi_points", 1);
    //ground_pub= node.advertise<sensor_msgs::PointCloud2>("ground_points", 1);

    //seg_pub= node.advertise<sensor_msgs::PointCloud2>("cluster_points", 1);
    //marker_pub = node.advertise<visualization_msgs::MarkerArray>("obstacle_boxes", 1);
    left_pub = node.advertise<sensor_msgs::PointCloud2>("left_points", 1);
    right_pub = node.advertise<sensor_msgs::PointCloud2>("right_points", 1);
    center_pub = node.advertise<sensor_msgs::PointCloud2>("center_points", 1);

    //subscribe
    data_sub=node.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",100,&Detection::dataCallback,this);    //3d
//    data_sub=node.subscribe<sensor_msgs::PointCloud2>("/cloud",100,&Detection::dataCallback,this);    //2d
    map_Flag = node.subscribe<std_msgs::Bool>("/track/track_path/path_exist",1,&Detection::map_Flag_CB, this);
    pnh.param<bool>("shutdown", shutdown, true);
}

void Detection::dataCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
//    sensor_msgs::PointCloud2 output;
    raw_data.clear();

//    projector.transformLaserScanToPointCloud("laser", *msg, output, listener);
    pcl::fromROSMsg(*msg,raw_data);
    Init();
}
void Detection::map_Flag_CB(const std_msgs::Bool::ConstPtr& msg) {
    map_state = *msg;
    if (map_state.data && shutdown){
        ros::shutdown();
    }
    ROS_INFO("%s", map_state.data ? "True" : "False");

}
void Detection::Init()
{
    //3d
    pcl::PointCloud<pcl::PointXYZI> way_point_filter;
    pcl::PointCloud<pcl::PointXYZI> left_filter;
    pcl::PointCloud<pcl::PointXYZI> right_filter;
    pcl::PointCloud<pcl::PointXYZI> center_filter;
    pcl::PointCloud<pcl::PointXYZI> roi_filter=RoiFilter(raw_data);
    pcl::PointCloud<pcl::PointXYZI> ground_filter=GroundFilter(roi_filter);
    pcl::PointCloud<pcl::PointXYZI> cluster_data=Clustering(ground_filter, &way_point_filter, &left_filter, &right_filter, &center_filter);
    //2d
//    pcl::PointCloud<pcl::PointXYZI> way_point_filter;
//    pcl::PointCloud<pcl::PointXYZI> left_filter;
//    pcl::PointCloud<pcl::PointXYZI> right_filter;
//    pcl::PointCloud<pcl::PointXYZI> roi_filter=RoiFilter(raw_data);
//    pcl::PointCloud<pcl::PointXYZI> cluster_data=Clustering(roi_filter, &way_point_filter, &left_filter, &right_filter);



    //visualization_msgs::MarkerArray obstacle_box= drawingBox();

    sensor_msgs::PointCloud2 waypoint_output;
    pcl::toROSMsg(way_point_filter,waypoint_output);
    waypoint_output.header.frame_id="velodyne";

//    sensor_msgs::PointCloud2 output;
//    pcl::toROSMsg(cluster_data,output);
//    output.header.frame_id="velodyne";

    sensor_msgs::PointCloud2 leftoutput;
    pcl::toROSMsg(left_filter,leftoutput);
    leftoutput.header.frame_id="velodyne";


    sensor_msgs::PointCloud2 rightoutput;
    pcl::toROSMsg(right_filter,rightoutput);
    rightoutput.header.frame_id="velodyne";

//    //3d
//    sensor_msgs::PointCloud2 ground_output;
//    pcl::toROSMsg(ground_filter,ground_output);
//    ground_output.header.frame_id="velodyne";

    sensor_msgs::PointCloud2 roi_output;
    pcl::toROSMsg(roi_filter,roi_output);
    roi_output.header.frame_id="velodyne";

    sensor_msgs::PointCloud2 cen_output;
    pcl::toROSMsg(center_filter,cen_output);
    cen_output.header.frame_id="velodyne";




    waypoint_pub.publish(waypoint_output);
//    seg_pub.publish(output);
    left_pub.publish(leftoutput);
    right_pub.publish(rightoutput);
//    //3d
//    ground_pub.publish(ground_output);
//    roi_pub.publish(roi_output);
    center_pub.publish(cen_output);

}
