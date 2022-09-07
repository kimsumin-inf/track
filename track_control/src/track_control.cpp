//
// Created by sumin on 22. 9. 6.
//

#include "track_control/track_control.h"

using namespace std;

Track_Control::Track_Control()
:nh(""), pnh("")
{
    pnh.param<double>("L", L, 0.1);
    pnh.param<double>("VL", VL, 1.6);
    pnh.param<double>("max_lfd", max_lfd, 10.0);
    pnh.param<double>("min_lfd", min_lfd, 1.4);    pnh.param<int>("hoped_vel", hoped_vel, 50);
    pnh.param<int>("circuit", circuit, 5);

    sub_Path = nh.subscribe("/track/track_path/global_path", 1, &Track_Control::path_CB, this);
    sub_Pose = nh.subscribe("/track/track_localizer/LocalPose",1,&Track_Control::pose_CB,this);
    sub_Map_state = nh.subscribe("/track/track_path/path_exist",1,&Track_Control::map_state_CB, this);
    sub_Init_Utm = nh.subscribe("/track/track_path/utm_init_pos", 1,&Track_Control::init_utm_CB,this);
    sub_Vel = nh.subscribe("/bestvel", 1, &Track_Control::vel_CB,this);

    pub_Cmd = nh.advertise<geometry_msgs::Twist>("current_pose", 1);
    pub_Tracking_Path = nh.advertise<nav_msgs::Path>("/track/track_control/tracking_path",1);

    Flag = false;

}
void Track_Control::vel_CB(const novatel_gps_msgs::NovatelVelocity::ConstPtr &msg) {
    gps_vel = *msg;
    real_vel = sqrt(pow(gps_vel.vertical_speed,2)+pow(gps_vel.horizontal_speed,2))*36;

    control_vel = int(hoped_vel+(hoped_vel - real_vel));
}
void Track_Control::path_CB(const nav_msgs::Path::ConstPtr &msg) {
    path =*msg;
    for(auto i : path.poses){

    }

}

void Track_Control::pose_CB(const geometry_msgs::PoseWithCovariance::ConstPtr &msg) {
    pose = *msg;
    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(erp_roll, erp_pitch, erp_yaw);
}

void Track_Control::map_state_CB(const std_msgs::Bool::ConstPtr &msg) {
    map_state = *msg;
    if (map_state.data == true){
        ROS_INFO("Path Generated, and Follow");
        Flag = true;
    }
    else {
        ROS_INFO("Path Not Generated, waiting for Path");
    }

}
void Track_Control::init_utm_CB(const geometry_msgs::Pose::ConstPtr &msg) {
    init_utm = *msg;

}

double Track_Control::pure_pursuit()
{
    is_look_foward_point = false;

    double back_x = pose.pose.position.x- init_utm.position.x - L*cos(erp_yaw);
    double back_y = pose.pose.position.y- init_utm.position.y - L*sin(erp_yaw);

    double dis = 0;

    double lfd = 1;
    double max_lfd = this->max_lfd;
    double min_lfd = this->min_lfd;
    double rotated_x = 0;
    double rotated_y = 0;

    lfd = 0.21/0.1;

    if(lfd < min_lfd)
    {
        lfd = min_lfd;
    }
    else if(lfd > max_lfd)
    {
        lfd = max_lfd;
    }

    for(int i = 0; i<tracking_path.poses.size(); i++)
    {
        double dx = tracking_path.poses.at(i).pose.position.x - back_x;
        double dy = tracking_path.poses.at(i).pose.position.y - back_y;

        rotated_x = cos(erp_yaw)*dx + sin(erp_yaw)*dy;
        rotated_y = -sin(erp_yaw)*dx + cos(erp_yaw)*dy;

        if(rotated_x > 0)
        {
            dis = sqrt(pow(rotated_x,2) + pow(rotated_y,2));
            if(dis>=lfd)
            {
                is_look_foward_point = true;
                break;
            }
        }
    }

    double theta = atan2(rotated_y,rotated_x);
    double steering = 0;
    if(is_look_foward_point == true)
    {
        double eta = atan2((2*VL*sin(theta)),lfd);
        steering = eta;
    }
    else
    {
        ROS_INFO("no found forwad point");
    }

    return steering;
}


void Track_Control::local_path(nav_msgs::Path path)
{
    if(Flag == true)
    {
        double least_dist = 10;
        int temp_num = 0;
        for(int i = 0; i<path.poses.size(); i++)
        {
            double dx = pose.pose.position.x - init_utm.position.x - path.poses.at(i).pose.position.x;
            double dy = pose.pose.position.y - init_utm.position.y - path.poses.at(i).pose.position.y;

            double dist = sqrt(dx*dx + dy*dy);
            if(dist<least_dist)
            {
                least_dist = dist;
                temp_num = i;
            }
        }
        tracking_path.header.stamp = ros::Time::now();
        tracking_path.header.frame_id = "map";
        tracking_path.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.header.frame_id = "map";

        ROS_INFO("now index: %d, total index: %d", temp_num, path.poses.size());

        for(int i = temp_num; i< temp_num + 20; i++)
        {
            if (i>=path.poses.size()){
                int j = i-path.poses.size();

                temp_pose.pose.position.x = path.poses.at(j).pose.position.x;
                temp_pose.pose.position.y = path.poses.at(j).pose.position.y;
                temp_pose.pose.position.z = 0;
                tracking_path.poses.push_back(temp_pose);
            }
            else {

                temp_pose.pose.position.x = path.poses.at(i).pose.position.x;
                temp_pose.pose.position.y = path.poses.at(i).pose.position.y;
                temp_pose.pose.position.z = 0;
                tracking_path.poses.push_back(temp_pose);
            }
        }

        pub_Tracking_Path.publish(tracking_path);

    }
}

void Track_Control::process() {
    if(Flag == true)
    {
        printf("\033[2J");
        printf("\033[1;1H");
        ROS_INFO("current_speed: %d KPH, hoped_speed: %d KPH, control_speed: %d KPH",real_vel/10, hoped_vel/10, control_vel/10);
        ROS_INFO("Steering value:%f", pure_pursuit());
        local_path(path);
        cmd_vel.linear.x = control_vel;
        cmd_vel.angular.z = pure_pursuit()*71;
        pub_Cmd.publish(cmd_vel);

    }
}