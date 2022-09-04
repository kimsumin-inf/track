//
// Created by sumin on 22. 9. 5.
//

#include "track_path/track_path.h"

using namespace std;
using namespace cv;

track_path::track_path()
:nh(""), pnh(""), loop_rate(10)
{
    pnh.param<double>("interval", interval, 0.5);

    sub_GPS = nh.subscribe("/bestpos",1, &track_path::gps_CB, this);
    pub_PATH_FLAG = nh.advertise<std_msgs::Bool>("/track/track_path/path_exist", 1);
    pub_PATH = nh.advertise<nav_msgs::Path>("/track/track_path/global_path", 1);
    pub_INIT_UTM = nh.advertise<geometry_msgs::Pose>("/track/track_path/utm_init_pos",1);
    pub_CIRCUIT_CNT = nh.advertise<std_msgs::Int16>("/track/track_path/circuit_count",1);

    //state param initialize
    gps_init_param = true;
    rising = false;
    falling = false;
    map_generate_param = false;

    //variable initialize
    now_distance = 0.;
    circuit_cnt = 0;


}
void track_path::gps_CB(const novatel_gps_msgs::NovatelPosition::ConstPtr &msg) {
    GPS = *msg;
    string utm_zone;
    RobotLocalization::NavsatConversions::LLtoUTM(GPS.lat,GPS.lon, now.x, now.y, utm_zone);
    if (gps_init_param == true){

        init.x = now.x;
        init.y = now.y;
        init_pos.position.x = init.x;
        init_pos.position.y = init.y;
        init_pos.position.z = 0;

        prev.x = now.x;
        prev.y = now.y;

        vec_utm.push_back(init);
        gps_init_param = false;
    }
    else {
        now_distance = calc_distance(prev, now);
        if (now_distance>= interval){
            printf("\033[2J");
            printf("\033[1;1H");
            now_state(calc_distance(now, init));
            prev = now;
            pub_INIT_UTM.publish(init_pos);
            cnt.data= circuit_cnt;
            pub_CIRCUIT_CNT.publish(cnt);
            global_path_flag.data = map_generate_param;
            pub_PATH_FLAG.publish(global_path_flag);
            show_map();
        }

    }

}

void track_path::now_state(double distance) {
    if (distance > interval){
        rising = true;
        if(map_generate_param == false){
            ROS_INFO("Map Generating");
            vec_utm.push_back(now);
            ROS_INFO("path_vec_size: %d", vec_utm.size());
        }
        else{
            ROS_INFO("Map Generated");
        }

        ROS_INFO("Distance : %lf", distance);
        ROS_INFO("Circuit: %d", circuit_cnt);
    }
    if(distance <= interval && rising==true){
        falling =true;
        rising = false;
        ROS_INFO("Start Point and End Point ard Adjacent.");

        if(map_generate_param == false){
            ROS_INFO("Map Generated");

            vec_utm.push_back(init);
            ROS_INFO("path_vec_size: %d", vec_utm.size());
            map_generate_param = true;

        }

    }
    if (rising == true && falling == true){
        ROS_INFO("Circuit Increase");
        circuit_cnt +=1;
        ROS_INFO("Circuit: %d", circuit_cnt);
        falling = false;
    }
}

void track_path::show_map(){
    ROS_INFO("Global Path Published");
    global_path.header.stamp = ros::Time::now();
    global_path.header.frame_id = "map";
    global_path.poses.clear();

    geometry_msgs::PoseStamped temp_pose;
    temp_pose.header.stamp = ros::Time::now();
    temp_pose.header.frame_id = "map";

    for(auto i : vec_utm){
        temp_pose.pose.position.x = i.x -init.x;
        temp_pose.pose.position.y = i.y -init.y;
        temp_pose.pose.position.z = 0;
        global_path.poses.push_back(temp_pose);
    }
    pub_PATH.publish(global_path);
    global_path.poses.clear();
}

double track_path::calc_distance(UTM pos1, UTM pos2) {
    return abs(sqrt(pow(pos2.x-pos1.x,2)+pow(pos2.y- pos1.y,2)));
}

