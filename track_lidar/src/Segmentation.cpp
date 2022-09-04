//
// Created by sumin on 22. 9. 4.
//

#include"track_lidar/Segmentation.h"

float distance_2(pcl::PointXYZI a, pcl::PointXYZI b){
    float x=std::abs(a.x-b.x);
    float y=std::abs(a.y-b.y);
    return std::sqrt(x*x+y*y);
}

float distance_1(pcl::PointXYZI a){
    return std::sqrt(a.x*a.x+a.y*a.y);
}

bool cmp_distance(const pcl::PointXYZI &a,const pcl::PointXYZI &b)
{
    return distance_1(a)<distance_1(b);
}

bool cmp_y(const pcl::PointXYZI &a,const pcl::PointXYZI &b)
{
    return a.y<b.y;
}

float make_angle(std::vector<struct NODE> left, std::vector<struct NODE> right, pcl::PointXYZI point, int direction){
    float angle;
    if(left.size()>=2 && right.size()>=2){
        angle=(left.at(1).slope+right.at(1).slope)*3.141592/360;
    }
    else{
        if(left.size()==1 && right.size()==2){
            angle=right.at(1).slope*3.141592/180;
        }
        else if(left.size()==2 && right.size()==1){
            angle=left.at(1).slope*3.141592/180;
        }
        else{
            angle=std::atan2(point.y, point.x);
        }
    }
    return angle;
}

pcl::PointXYZI make_tempway(float angle, pcl::PointXYZI point){
    pcl::PointXYZI tempway;
    tempway.x=-2.*std::cos(std::abs(angle))+point.x;
    if(angle>0)
        tempway.y=-1.*std::sin(std::abs(angle))+point.y;
    else
        tempway.y=1.*std::sin(std::abs(angle))+point.y;
    return tempway;
}

void graph_input(std::vector<struct NODE>& graph, struct NODE& node, float slope){
    node.slope=slope;
    graph.push_back(node);
}

Segmentation::Segmentation()
{
    // cluster value
    cluster_min = 0;
    cluster_max = 10;
    cluster_thresh = 0.7;
    pre_waypoint.x=-1;
}

void Segmentation::Graph_Input(bool left, bool right, NODE& node, float left_slope, float right_slope){
    if(left==true && right==true){      //left & right = true : 거리가 가까운 것을 우선적으로
        float left_distance=distance_2(node.point, left_graph.at(left_graph.size()-1).point);
        float right_distance=distance_2(node.point, right_graph.at(right_graph.size()-1).point);
        if(left_distance>right_distance){
            graph_input(right_graph, node, right_slope);
        }
        else{
            graph_input(left_graph, node, left_slope);
        }
    }
    else if(left==true && right==false){
        graph_input(left_graph, node, left_slope);
    }
    else if(left==false && right==true){
        graph_input(right_graph, node, right_slope);
    }
    else{
        float left_angle=std::atan2(node.point.y-left_graph.at(left_graph.size()-1).point.y,node.point.x-left_graph.at(left_graph.size()-1).point.x)*180/3.141592;
        float right_angle=std::atan2(node.point.y-right_graph.at(right_graph.size()-1).point.y,node.point.x-right_graph.at(right_graph.size()-1).point.x)*180/3.141592;
        if(direction==1){
            if(std::abs(left_angle)<60 || std::abs(right_angle)<60){
                if(std::abs(left_angle)<std::abs(right_angle)){
                    graph_input(left_graph, node, left_slope);
                }
                else{
                    graph_input(right_graph, node, right_slope);
                }
            }
        }
        else{
            bool small=false;
            bool big=false;
            if(std::abs(dir_slope)<30.){
                if(std::abs(left_angle)<std::abs(right_angle)){
                    small=true;
                }
            }
            else{
                if(std::abs(left_angle)>std::abs(right_angle)){
                    big=true;
                }
            }
            if(direction==2){
                if(right_angle>0 && left_angle<0 && right_graph.at(right_graph.size()-1).point.y-0.5<=node.point.y){
                    graph_input(right_graph, node, right_slope);
                }
                else if(left_angle>0 && right_angle<0 && left_graph.at(left_graph.size()-1).point.y-0.5<=node.point.y){
                    graph_input(left_graph, node, left_slope);
                }
                else{
                    if((small==true || big==true) && left_graph.at(left_graph.size()-1).point.y-0.5<=node.point.y){
                        graph_input(left_graph, node, left_slope);
                    }
                    else{
                        if(right_graph.at(right_graph.size()-1).point.y-0.5<=node.point.y){
                            graph_input(right_graph, node, right_slope);

                        }
                    }
                }
            }
            else{

                if(right_angle<0 && left_angle>0 && right_graph.at(right_graph.size()-1).point.y-0.5>=node.point.y){
                    graph_input(right_graph, node, right_slope);
                }
                else if(left_angle<0 && right_angle>0 && left_graph.at(left_graph.size()-1).point.y-0.5>=node.point.y){
                    graph_input(left_graph, node, left_slope);
                }
                else{
                    if((small==true || big==true) && left_graph.at(left_graph.size()-1).point.y-0.5>=node.point.y){
                        graph_input(left_graph, node, left_slope);
                    }
                    else{
                        if(right_graph.at(right_graph.size()-1).point.y-0.5>=node.point.y){
                            graph_input(right_graph, node, right_slope);
                        }
                    }
                }
            }
        }
    }
}

void Segmentation::Make_Graph(int left, int right){

    float min_x=left_graph.at(0).point.x;
    if(left_graph.at(0).point.x>right_graph.at(0).point.x){
        min_x=right_graph.at(0).point.x;
    }

    for(int i=0; i<cen_point.size(); i++){
        float left_distance=distance_2(left_graph.at(left_graph.size()-1).point, cen_point.at(i));
        float right_distance=distance_2(right_graph.at(right_graph.size()-1).point, cen_point.at(i));
        bool left_con_value=false;
        bool right_con_value=false;
        left_graph.at(0).slope=std::atan2(left_graph.at(0).point.y,left_graph.at(0).point.x)*180/3.141592;
        right_graph.at(0).slope=std::atan2(right_graph.at(0).point.y,right_graph.at(0).point.x)*180/3.141592;

        if(i!=left && i!=right && cen_point.at(i).x>min_x-0.5 && (right_distance<5. || left_distance<5.)&& (right_distance>2. || left_distance>2.)){
            float left_diff_x = std::abs(left_graph.at(left_graph.size()-1).point.x-cen_point.at(i).x);
            float left_diff_y = std::abs(left_graph.at(left_graph.size()-1).point.y-cen_point.at(i).y);
            float right_diff_x = std::abs(right_graph.at(right_graph.size()-1).point.x-cen_point.at(i).x);
            float right_diff_y = std::abs(right_graph.at(right_graph.size()-1).point.y-cen_point.at(i).y);
            float left_slope_angle=std::atan2(cen_point.at(i).y-left_graph.at(left_graph.size()-1).point.y,cen_point.at(i).x-left_graph.at(left_graph.size()-1).point.x)*180/3.141592;
            float right_slope_angle=std::atan2(cen_point.at(i).y-right_graph.at(right_graph.size()-1).point.y,cen_point.at(i).x-right_graph.at(right_graph.size()-1).point.x)*180/3.141592;

            NODE node;
            node.point.x=cen_point.at(i).x;
            node.point.y=cen_point.at(i).y;
//            node.angle=std::atan2(cen_point.at(i).y,cen_point.at(i).x)*180/3.141592;

            if(direction==1){   //STRAIGHT

                if(left_diff_y<1. && left_graph.at(left_graph.size()-1).point.x<cen_point.at(i).x){
                    left_con_value=true;
                }
                if(right_diff_y<1. && right_graph.at(right_graph.size()-1).point.x<cen_point.at(i).x){
                    right_con_value=true;
                }
                Graph_Input(left_con_value, right_con_value, node, left_slope_angle, right_slope_angle);

            }
            else if(direction ==2){     //LEFT

                if(cen_point.at(i).y>right_graph.at(right_graph.size()-1).point.y-0.5){

                    if(left_diff_y<4. && left_diff_x<4. && right_graph.at(right_graph.size()-1).slope/2<left_slope_angle && left_graph.at(left_graph.size()-1).point.y-0.5<=cen_point.at(i).y && left_graph.at(left_graph.size()-1).point.x-0.5<=cen_point.at(i).x){
                        left_con_value=true;
                    }
                    if(right_diff_y<5. && right_diff_x<5.5 && right_graph.at(right_graph.size()-1).slope/2<right_slope_angle && left_graph.at(left_graph.size()-1).slope/2>right_slope_angle && right_graph.at(right_graph.size()-1).point.x-0.5<=cen_point.at(i).x){
                        right_con_value=true;
                    }
                    Graph_Input(left_con_value, right_con_value, node, left_slope_angle, right_slope_angle);

                }
            }
            else{       //RIGHT
                if(cen_point.at(i).y<left_graph.at(left_graph.size()-1).point.y+0.5){

                    if(left_diff_y<5. && left_diff_x<5.5 && left_graph.at(left_graph.size()-1).slope/2>left_slope_angle && right_graph.at(right_graph.size()-1).slope/2<left_slope_angle && left_graph.at(left_graph.size()-1).point.x-0.5<cen_point.at(i).x){
                        left_con_value=true;
                    }
                    if(right_diff_y<4. && right_diff_x<4. && left_graph.at(left_graph.size()-1).slope/2>right_slope_angle&& right_graph.at(right_graph.size()-1).point.y+0.5>cen_point.at(i).y && right_graph.at(right_graph.size()-1).point.x-0.5<cen_point.at(i).x){
                        right_con_value=true;
                    }
                    Graph_Input(left_con_value, right_con_value, node, left_slope_angle, right_slope_angle);
                }
            }
        }
    }
}

void Segmentation::Make_Direction_Before(){

    float sumy=0;
    float y_mean;
    for(int i=0; i<cen_point.size(); i++){
        sumy+=cen_point.at(i).y;
    }
    y_mean=sumy/cen_point.size();
    if(std::abs(y_mean)<1.){
        direction=1;
    }
    else{
        if(y_mean>0){
            direction=2;
        }
        else{
            direction=3;
        }
    }
    class_count++;

}

void Segmentation::Make_Direction_After(){
    float final_l=left_point.at(left_point.size()-1).y;
    float final_r=right_point.at(right_point.size()-1).y;
    float final_y=(final_l+final_r)/2;
    if(std::abs(final_y)<2.){
        direction=1;
    }
    else{
        if(final_y>0){
            direction=2;
        }
        else{
            direction=3;
        }
    }
}

void Segmentation::First_Point_Input(int left_or_right, pcl::PointXYZI point, bool& value){
    int cen;
    std::sort(cen_point.begin(), cen_point.end(), cmp_distance);
    for(int i=0; i<cen_point.size(); i++){
        if(cen_point.at(i).x==point.x && cen_point.at(i).y==point.y){
            cen=i;
        }
    }
    if(left_or_right==1){
        left_point.push_back(point);
        if(value==false){
            value=true;
            left_cen=cen;
        }
    }
    else{
        right_point.push_back(point);
        if(value==false){
            value=true;
            right_cen=cen;
        }
    }
}

void Segmentation::First_Point_False(bool& left, bool& right){
    std::sort(cen_point.begin(), cen_point.end(), cmp_y);
    if(left==false){
        if(direction==2){
            direction=1;
        }
        else{
            if(direction==3){
                right=false;
                right_point.clear();
            }
            First_Point_Input(1, cen_point.at(cen_point.size()-1), left);
        }
    }
    if(right==false){
        if(direction==3){
            direction=1;
        }
        else{
            if(direction==2){
                left=false;
                left_point.clear();
            }
            First_Point_Input(2, cen_point.at(0), right);
        }
    }
}

void Segmentation::Classification(){

    bool left_value=false;
    bool right_value=false;

    if(cen_point.size()>1){
        if(class_count==0){
            Make_Direction_Before();
        }
        while(left_value==false || right_value==false){
            if(direction == 1){ //STRAIGHT
                for(int i=0; i<cen_point.size(); i++){
                    if(std::abs(cen_point.at(i).y)<4. && cen_point.at(i).x<8.){
                        if(cen_point.at(i).y>0){
                            First_Point_Input(1, cen_point.at(i), left_value);
                        }
                        else{
                            First_Point_Input(2, cen_point.at(i), right_value);
                        }
                    }
                }
                if(left_value==false || right_value==false){
                    First_Point_False(left_value, right_value);
                }
            }
            else if(direction == 2){    //LEFT
                for(int i=0; i<cen_point.size(); i++){
                    if(cen_point.at(i).y>-4. && cen_point.at(i).y<5.&& cen_point.at(i).x<8.){
                        if(cen_point.at(i).y>0.5){
                            First_Point_Input(1, cen_point.at(i), left_value);
                        }
                        else{
                            First_Point_Input(2, cen_point.at(i), right_value);
                        }
                    }
                }
                if(left_value==false || right_value==false){
                    First_Point_False(left_value, right_value);
                }
            }
            else{    //RIGHT
                for(int i=0; i<cen_point.size(); i++){
                    if(cen_point.at(i).y<4. && cen_point.at(i).y>-5.&& cen_point.at(i).x<8.){
                        if(cen_point.at(i).y>-0.5){
                            First_Point_Input(1, cen_point.at(i), left_value);
                        }
                        else{
                            First_Point_Input(2, cen_point.at(i), right_value);
                        }
                    }
                }
                if(left_value==false || right_value==false){
                    First_Point_False(left_value, right_value);
                }
            }

        }






        NODE left_node, right_node;
        if(left_point.size()>0 && right_point.size()>0){
            left_node.point.x=cen_point.at(left_cen).x;
            left_node.point.y=cen_point.at(left_cen).y;
            right_node.point.x=cen_point.at(right_cen).x;
            right_node.point.y=cen_point.at(right_cen).y;
            left_graph.push_back(left_node);
            right_graph.push_back(right_node);
            left_point.clear();
            right_point.clear();


            Make_Graph(left_cen, right_cen);

            for(int i=0; i<right_graph.size(); i++){
                right_point.push_back(right_graph.at(i).point);
            }
            for(int i=0; i<left_graph.size(); i++){
                left_point.push_back(left_graph.at(i).point);
            }

        }
        else{
            std::cout << "not first point" << std::endl;
        }
    }
    else{
        std::cout << "not center point" << std::endl;
    }


}

void Segmentation::Make_Waypoint(){

    pcl::PointXYZI tempway;
    pcl::PointXYZI current_cen;
//    pcl::PointXYZI pre_cen;
//    pcl::PointXYZI final_cen;
    float current_angle;
//    float final_angle;
//    float pre_angle;
//    pcl::PointXYZI pre_first_cen;
//    int cen_count=0;
    int min;
    std::vector<pcl::PointXYZI> cen;

    min=left_point.size();
    if(min>right_point.size()){
        min=right_point.size();
    }

    if(min>=1){
        current_cen.x=(left_point.at(0).x+right_point.at(0).x)/2;
        current_cen.y=(left_point.at(0).y+right_point.at(0).y)/2;
        current_angle=make_angle(left_graph, right_graph, current_cen, direction);

        if(std::abs(current_angle)<0.12){
            current_cen.x=(left_point.at(0).x+right_point.at(0).x)/2;
            current_cen.y=(left_point.at(0).y+right_point.at(0).y)/2;
        }
        else{
            if(current_angle>0){
                current_cen.x=(left_point.at(0).x+right_point.at(0).x)/2;
                current_cen.y=right_point.at(0).y+((std::abs(left_point.at(0).y)+std::abs(right_point.at(0).y))/3);
            }
            else{
                current_cen.x=(left_point.at(0).x+right_point.at(0).x)/2;
                current_cen.y=left_point.at(0).y-((std::abs(left_point.at(0).y)+std::abs(right_point.at(0).y))/3);
            }
        }


        std::cout << current_cen.x<< " "<<current_cen.y << " : " << current_angle ;

        if(way_value==false){
            way_value=true;
            final_angle=current_angle;
            pre_angle=current_angle;

        }
        else{
            if(pre_cen.x+1<current_cen.x){
                final_angle=pre_angle;
                std::cout << "=> change => " << final_angle << std::endl;
            }
            else{
                std::cout << " => " << final_angle << std::endl;
            }
        }
//        next_angle=current_angle;
        pre_angle=current_angle;
        pre_cen.x=current_cen.x;

        tempway=make_tempway(final_angle, current_cen);



//        std::cout << "tempway : " << tempway.x<< " "<<tempway.y << std::endl;

        double a=tempway.y/tempway.x;
        double b=tempway.y-a*tempway.x;
        float way_con_d;



        if(direction==2){
            for(int i=0; i<left_point.size(); i++){
                way_con_d=std::abs(a*left_point.at(i).x-left_point.at(i).y+b)/(std::sqrt(a*a+1));
                if(way_con_d<0.8){
//                    std::cout << "not =>";
                    final_angle=final_angle-0.1;
                    tempway=make_tempway(final_angle, current_cen);
//                    std::cout << angle << ": " << tempway.x<< " "<<tempway.y << std::endl;
                }
            }
        }
        if(direction==3){
            for(int i=0; i<right_point.size(); i++){
                way_con_d=std::abs(a*right_point.at(i).x-right_point.at(i).y+b)/(std::sqrt(a*a+1));

                if(way_con_d<0.8){
//                    std::cout << "not =>";
                    final_angle=final_angle+0.1;
                    tempway=make_tempway(final_angle, current_cen);
//                    std::cout << angle << ": " << tempway.x<< " "<<tempway.y << std::endl;
                }

            }
        }


        way_point.push_back(tempway);
//        way_point.push_back(first_cen);

    }
    else{
        std::cout<<"not find labacon"<<std::endl;
    }


}

pcl::PointCloud<pcl::PointXYZI> Segmentation::Clustering(pcl::PointCloud<pcl::PointXYZI> before_filter, pcl::PointCloud<pcl::PointXYZI> *way_point_filter,pcl::PointCloud<pcl::PointXYZI> *left_filter,pcl::PointCloud<pcl::PointXYZI> *right_filter, pcl::PointCloud<pcl::PointXYZI> *center_filter)
{

    float min_x,max_x,min_y,max_y,min_z,max_z;
    float init=0.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> after_filter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr way_point_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr left_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr right_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr center_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    *cloud=before_filter;

    if(cloud->size()>0)
    {
        cloud=cloud->makeShared();
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance (cluster_thresh);
        ec.setMinClusterSize (cluster_min);
        ec.setMaxClusterSize (cluster_max);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);
        cv::Point2f min_point;

        int j = 0;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pt (new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            cloud_pt->clear();
            init=0;
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            {
                pcl::PointXYZI pt= cloud->points[*pit];
                cloud->points[*pit].PointXYZI::intensity=j;
                cloud_filtered->push_back(cloud->points[*pit]);
                cloud_pt->push_back(cloud->points[*pit]);

                if(init==0.0)
                {
                    min_x=pt.x;
                    min_y=pt.y;
                    max_x=pt.x;
                    max_y=pt.y;
                    init=1.;
                }
                else
                {
                    if(min_x > pt.x)
                    {
                        min_x=pt.x;
                    }
                    if(min_y>pt.y)
                    {
                        min_y=pt.y;
                    }
                    if(max_x < pt.x)
                    {
                        max_x=pt.x;
                    }
                    if(max_y < pt.y)
                    {
                        max_y=pt.y;
                    }
                }


            }
            float height = std::abs(max_x-min_x);
            float width =std::abs(max_y-min_y);

            pcl::PointXYZI c_point;
            c_point.x=max_x-(height/2);
            c_point.y=max_y-(width/2);
            float thresh_x=15*std::sqrt(1-c_point.y*c_point.y/25);
            if(c_point.x<thresh_x){
                cen_point.push_back(c_point);
            }

            j++;
        }

        if(cen_point.size()>0){
            std::sort(cen_point.begin(),cen_point.end(),cmp_distance);
            Make_Direction_Before();
            Classification();
            Make_Waypoint();
            Make_Direction_After();


            for(int i=0; i<left_point.size(); i++){
                left_point.at(i).PointXYZI::intensity=30;
                left_filtered->push_back(left_point.at(i));
            }
            *left_filter=*left_filtered;
            left_point.clear();
            left_graph.clear();

            for(int i=0; i<right_point.size(); i++){
                right_point.at(i).PointXYZI::intensity=40;
                right_filtered->push_back(right_point.at(i));
            }
            right_point.clear();
            *right_filter=*right_filtered;
            right_graph.clear();

            for(int i=0; i<way_point.size(); i++){
                way_point.at(i).PointXYZI::intensity=50;
                way_point_filtered->push_back(way_point.at(i));
            }
            way_point.clear();
            *way_point_filter=*way_point_filtered;

            for(int i=0; i<cen_point.size(); i++){
                cen_point.at(i).PointXYZI::intensity=30;
                center_filtered->push_back(cen_point.at(i));
            }
            *center_filter=*center_filtered;
            cen_point.clear();


            after_filter=*cloud_filtered;
            return after_filter;
        }
        else{
            std::cout << "no center point" << std::endl;

        }
    }
    else
    {
        std::cerr<<"no data"<<std::endl;
    }

}