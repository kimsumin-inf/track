//
// Created by sumin on 22. 9. 6.
//

#include "track_control/track_control.h"

int main(int argc, char**argv){
    ros::init(argc,argv,"track_control");
    Track_Control TC;
    ros::Rate loop_rate(10);
    while(ros::ok()){
        TC.process();
        loop_rate.sleep();
        ros::spinOnce();

    }
    ros::spin();
    return 0;
}