//
// Created by sumin on 22. 9. 4.
//

#include "track_lidar/Filtering.h"


Filtering::Filtering()
{
    //roi filter value
    cv::Point3d distance(15., 6., 4.);

    max_distance.x = distance.x;
    max_distance.y = distance.y;
    max_distance.z= distance.z;

    min_distance.x = -0.5;
    min_distance.y = -distance.y;
    min_distance.z=-distance.z;

    height_thresh=0.1;
    grid_dim=320;
    per_cell=0.25;
}


pcl::PointCloud<pcl::PointXYZI> Filtering::RoiFilter(pcl::PointCloud<pcl::PointXYZI> before_filter)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> after_filter;

    *cloud=before_filter;

    if(cloud->size()>0)
    {
        // Create the filtering object
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (min_distance.x, max_distance.x);
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (min_distance.y,  max_distance.y);
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (min_distance.z,  max_distance.z);
        pass.filter (*cloud_filtered);

        after_filter=*cloud_filtered;
        return after_filter;
    }
    else
    {
        std::cerr<<"no data"<<std::endl;
    }

}

pcl::PointCloud<pcl::PointXYZI> Filtering::GroundFilter(pcl::PointCloud<pcl::PointXYZI> before_filter){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> after_filter;

    *cloud=before_filter;
    size_t cloud_size=cloud->points.size();

    cloud_filtered->points.resize(cloud_size);

    float min[grid_dim][grid_dim];
    float max[grid_dim][grid_dim];
    bool init[grid_dim][grid_dim];
    size_t obstacle_count=0;

    memset(&min, 0.0f, grid_dim*grid_dim*sizeof(float));
    memset(&max, 0.0f, grid_dim*grid_dim*sizeof(float));
    memset(&init, false, grid_dim*grid_dim*sizeof(bool));

    if(cloud->size()>0)
    {
        // build height map
        for (unsigned i = 0; i < cloud_size; ++i)
        {
            int x = ((grid_dim/2)+cloud->points[i].x/per_cell);
            int y = ((grid_dim/2)+cloud->points[i].y/per_cell);


            if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim) {
                if (!init[x][y])
                {
                    min[x][y] = cloud->points[i].z;
                    max[x][y] = cloud->points[i].z;
                    init[x][y] = true;
                }
                else
                {
                    min[x][y] = MIN(min[x][y], cloud->points[i].z);
                    max[x][y] = MAX(max[x][y], cloud->points[i].z);
                }
            }
        }


        // display points where map has height-difference > threshold
        double grid_offset=grid_dim/2.0*per_cell;
        for(int x=0; x<grid_dim; x++)
            for(int y=0; y<grid_dim; y++){
                if ((max[x][y] - min[x][y] )> height_thresh && max[x][y]<0)
                {
                    cloud_filtered->points[obstacle_count].x = -grid_offset + (x*per_cell+per_cell/2.0);
                    cloud_filtered->points[obstacle_count].y = -grid_offset + (y*per_cell+per_cell/2.0);
                    cloud_filtered->points[obstacle_count].z = height_thresh;
                    obstacle_count++;
                }
            }


        cloud_filtered->points.resize(obstacle_count);
        after_filter=*cloud_filtered;
        return after_filter;
    }
    else
    {
        std::cerr<<"no data"<<std::endl;
    }

}
