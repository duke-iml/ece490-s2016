#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
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
    fprintf(stderr, "Usage: \"program arg1 arg2 [arg3]\" where arg1 is the ply model file, arg2 is the sensed point cloud file and arg3 (optional) is the file to save aligned point_cloud of the model\n");
    exit(1);
  }
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGB>); // the point cloud obtained from UCB scanned model
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr real(new pcl::PointCloud<pcl::PointXYZRGB>); // the real point cloud processed from RealSense
  
  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[2], *real) == -1)
  {
    PCL_ERROR ("Couldn't read file %s\n", argv[2]);
    return (-1);
  }
  
  // std::cout << real->points.size() << "\n";
  
  pcl::PolygonMesh model_mesh;
  pcl::io::loadPolygonFilePLY(argv[1], model_mesh);
  pcl::fromPCLPointCloud2(model_mesh.cloud, *model);
  
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputSource(model);
  icp.setInputTarget(real);
  pcl::PointCloud<pcl::PointXYZRGB> transformed_model;
  icp.align(transformed_model);
  
  std::cout << icp.getFinalTransformation() << std::endl;
  
  ofstream f;
  f.open("perception/single_utils/xfrom_matrix.txt");
  f << icp.getFinalTransformation() << "\n";
  f.close();
  
  if(argc>=4) {
    pcl::io::savePCDFileASCII(argv[3], transformed_model);
  } else {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > ptr ( &transformed_model );
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_model(ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> (ptr, rgb_model, "model");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_real(real);
    viewer->addPointCloud<pcl::PointXYZRGB> (real, rgb_real, "real");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "real");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while(!viewer->wasStopped()) {
      viewer->spinOnce();
    }
  }

  return 0;
}


