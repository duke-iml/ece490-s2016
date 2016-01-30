#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>

#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace std;


int main(int argc, char **argv)
{
  if(argc < 3) {
    fprintf(stderr, "Usage: \"program arg1 arg2\" where arg1 is the ply model file, arg2 is the name of the output pcd file\n");
    exit(1);
  }
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGB>); // the point cloud obtained from UCB scanned model
  
  pcl::PolygonMesh model_mesh;
  pcl::io::loadPolygonFilePLY(argv[1], model_mesh);
  pcl::fromPCLPointCloud2(model_mesh.cloud, *model);
  pcl::io::savePCDFileASCII(argv[2], *model);

  return 0;
}


