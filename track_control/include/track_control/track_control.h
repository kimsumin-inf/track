//
// Created by sumin on 22. 9. 6.
//

#ifndef SRC_TRACK_CONTROL_H
#define SRC_TRACK_CONTROL_H

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Twist.h>

#include <visualization_msgs/MarkerArray.h>

#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <novatel_gps_msgs/NovatelVelocity.h>

class Track_Control{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber sub_Path;
    ros::Subscriber sub_Pose;
    ros::Subscriber sub_Map_state;
    ros::Subscriber sub_Init_Utm;
    ros::Subscriber sub_Vel;

    ros::Publisher pub_Cmd;
    ros::Publisher pub_Tracking_Path;

    void pose_CB(const geometry_msgs::PoseWithCovariance::ConstPtr &msg);
    void path_CB(const nav_msgs::Path::ConstPtr &msg);
    void map_state_CB(const std_msgs::Bool::ConstPtr &msg);
    void init_utm_CB(const geometry_msgs::Pose::ConstPtr & msg);
    void vel_CB(const novatel_gps_msgs::NovatelVelocity::ConstPtr& msg);
    double pure_pursuit();
    void local_path(nav_msgs::Path path);


    nav_msgs::Path path;
    nav_msgs::Path tracking_path;
    geometry_msgs::PoseWithCovariance pose;
    std_msgs::Bool map_state;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Pose init_utm;
    novatel_gps_msgs::NovatelVelocity gps_vel;

    double max_lfd, min_lfd;
    double VL, L;
    double erp_yaw, erp_roll, erp_pitch;
    int hoped_vel;
    int control_vel;

    bool Flag;
    bool is_look_foward_point;

    int circuit;

public:
    Track_Control();
    void process();
};

#endif //SRC_TRACK_CONTROL_H
