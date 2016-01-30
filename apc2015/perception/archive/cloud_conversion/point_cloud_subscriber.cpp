#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>



//pcl::PCLPointCloud2 cloud;
//pcl::PointCloud<pcl::PointXYZ> cloud;

void pointCloud2Callback(sensor_msgs::PointCloud2& pc2) {
    
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::moveFromPCL(pcl_pc, pc2);
}


int main(int argc, char **argv) {
    /*ros::init(argc, argv, "point_cloud_2_to_pcl");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/realsense/pc", 1000, pointCloud2Callback);
    ros::spin();
    return 0;*/
}

