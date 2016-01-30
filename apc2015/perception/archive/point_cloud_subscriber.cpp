#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/include <pcl/conversions.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>


//pcl::PCLPointCloud2 cloud;
//pcl::PointCloud<pcl::PointXYZ> cloud;

void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& input) {
    std::cout << "Received " << input->height << " " << input->width << "\n";
    pcl::PCLPointCloud2::Ptr cloud = input;
    //pcl::fromPCLPointCloud2(input, cloud);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "point_cloud_2_to_pcl");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/realsense/pc", 1000, pointCloud2Callback);
    ros::spin();
    return 0;
}

