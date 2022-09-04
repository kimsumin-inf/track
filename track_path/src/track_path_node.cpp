//
// Created by sumin on 22. 9. 5.
//

#include"track_path/track_path.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"track_path");
    track_path tp;
    ros::spin();
    return 0;
}
