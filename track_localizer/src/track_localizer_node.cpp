//
// Created by sumin on 22. 9. 5.
//
#include"track_localizer/track_localizer.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"track_localizer");
    Localizer TL;
    ros::spin();
    return 0;
}
