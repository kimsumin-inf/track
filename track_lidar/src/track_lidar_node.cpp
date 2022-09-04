//
// Created by sumin on 22. 9. 4.
//
#include"track_lidar/Detection.h"

int main(int argc, char **argv)
{

    ros::init(argc,argv,"track_lidar");
    ros::NodeHandle node;
    ros::NodeHandle pnh;
    Detection detect(node, pnh);
    ros::spin();
    return 0;
}
