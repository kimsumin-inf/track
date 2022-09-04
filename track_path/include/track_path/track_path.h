//
// Created by sumin on 22. 9. 5.
//

#ifndef SRC_TRACK_PATH_H
#define SRC_TRACK_PATH_H

#include <ros/ros.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

#include <cmath>

#include "track_path/navsat_conversions.h"

struct UTM{
    double x;
    double y;
    UTM operator-(UTM pos){
        UTM tmp;
        tmp.x = x- pos.x;
        tmp.y = y-pos.y;
        return tmp;
    }
};

class track_path{

private:
    void gps_CB(const novatel_gps_msgs::NovatelPosition::ConstPtr &msg);


    //ros variable
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Rate loop_rate;

    ros::Publisher pub_PATH;
    ros::Publisher pub_PATH_FLAG;
    ros::Publisher pub_INIT_UTM;
    ros::Publisher pub_CIRCUIT_CNT;

    ros::Subscriber sub_GPS;

    //Pos
    UTM init;
    UTM now;
    UTM prev;

    //msg data
    novatel_gps_msgs::NovatelPosition GPS;
    nav_msgs::Path global_path;
    geometry_msgs::Pose init_pos;
    std_msgs::Bool global_path_flag;
    std_msgs::Int16 cnt;
    std::vector<UTM> vec_utm;

    //state param
    bool gps_init_param;
    bool rising;
    bool falling;
    bool map_generate_param;


    // variable
    double now_distance;
    double interval;
    int circuit_cnt;

    // function
    double calc_distance(UTM pos1, UTM pos2);
    void now_state(double distance);
    void show_map();

public:
    track_path();
};

#endif //SRC_TRACK_PATH_H
