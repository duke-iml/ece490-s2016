#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("point_cloud.pcd", *cloud) == -1) {
    std::cout << "Error reading point cloud file point_cloud.pcd\n";
    return -1;
  }

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  ne.setRadiusSearch(0.05);
  ne.setViewPoint(0, 0, 0);

  ne.compute(*cloud_normals);
  
  //pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  //viewer.setBackgroundColor (0.0, 0.0, 0.0);
  //viewer.addCoordinateSystem(1, 0, 0, 0, "rgbcloud", 0);
  //viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "rgbcloud");
  //viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, cloud_normals, 20, 0.2, "normals");
  //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rgbcloud");
  //viewer.setCameraPosition(0, 0, 0, 0, 0, 1, 0, 0, 0);
  
  ofstream file;
  file.open("point_normals.txt");
  for(int i=0; i<cloud_normals->points.size(); i++) {
    pcl::Normal cur = cloud_normals->points[i];
    if(isnan(cur.normal_x)) {
      printf("isnan\n");
      continue;
    }
    file << cur.normal_x << " " << cur.normal_y << " " << cur.normal_z << "\n";
  }
  file.close();
  std::cout << "Successfully wrote to file!\n";
  
  //while(!viewer.wasStopped()) {
  //  viewer.spinOnce();
  //}
  
  return 0;
}
