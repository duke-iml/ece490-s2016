/*
 * convert2pcl.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: bl109
 */
 
// Usage: ./path/to/program model.ply object_segmented.pcd aligned.pcd xform_matrix.txt 
// where aligned.pcd and xform_matrix.txt are files to be created

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <string>
#include <iostream>
#include <cmath>

using namespace std;

typedef map<const string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PointCloudMap;
typedef PointCloudMap::iterator it_PCLMap;

bool transformCloud(double theta, char axis, double x, double y, double z, pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud)
{
	    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        // Define the translation
        transform.translation() << x, y, z;

        switch( axis ) {
		case 'x':
                 // The same rotation matrix as before; tetha radians arround X axis
		 transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
		 break;
		case 'y':
		 // The same rotation matrix as before; tetha radians arround Y axis
		 transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
		 break;
		case 'z':
  		 // The same rotation matrix as before; tetha radians arround Z axis
  		 transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
		 break;
	}

  	// Print the transformation
  	//printf ("\nTransformation matrix:\n");
  	//std::cout << transform.matrix() << std::endl;

  	// Executing the transformation
  	
 	 pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);

	return true;
}

double icp_ext( pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, 
    Eigen::Matrix4f & transform, char *xform_file_name )
{
	double curr_score, score;
	Eigen::Matrix4f transform1; 
	Eigen::Matrix3f transform2, curr_transform;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr t_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  
    bool reciprocal = false;
    
	icp.setInputSource(scene);
	icp.setInputTarget(model);
	icp.setUseReciprocalCorrespondences(reciprocal);	
	pcl::PointCloud<pcl::PointXYZRGB> Final;

	icp.align(Final);
	//std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
	//std::cout << icp.getFinalTransformation() << std::endl;

	pcl::transformPointCloud (*scene, *cloud_out, icp.getFinalTransformation());
        score = icp.getFitnessScore();

	*t_cloud = *model; 

	transform1 = icp.getFinalTransformation();
	transform2 << 1, 0, 0,  
	              0, 1, 0, 
	              0, 0, 1;

    int delta = 8;

	for( int i=1; i<=delta; i++ )
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		transformCloud(M_PI/delta*i, 'x', 0, 0, 0, model, transformed_cloud);

        curr_transform = Eigen::AngleAxisf (M_PI/delta*i, Eigen::Vector3f::UnitX());
        
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		icp.setInputSource(scene);
		icp.setInputTarget(transformed_cloud);
		icp.setUseReciprocalCorrespondences(reciprocal);
		pcl::PointCloud<pcl::PointXYZRGB> Final;

		icp.align(Final);
	//	std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
	//	std::cout << icp.getFinalTransformation() << std::endl;

         curr_score = icp.getFitnessScore();
		if( score > curr_score )
		{
            score = curr_score; 
			
			pcl::transformPointCloud (*scene, *cloud_out, icp.getFinalTransformation());
			
			transform1 = icp.getFinalTransformation();
            transform2 = curr_transform;
    
			
			*t_cloud = *transformed_cloud;
			transformed_cloud.reset();
		}
	
	}

	for( int i=1; i<=delta; i++ )
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		transformCloud(M_PI/delta*i, 'x', 0, 0, 0, model, transformed_cloud);
        curr_transform = Eigen::AngleAxisf (M_PI/delta*i, Eigen::Vector3f::UnitX());
        
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		icp.setInputSource(scene);
		icp.setInputTarget(transformed_cloud);
		icp.setUseReciprocalCorrespondences(reciprocal);
		pcl::PointCloud<pcl::PointXYZRGB> Final;

		icp.align(Final);
    	//std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
    	//std::cout << icp.getFinalTransformation() << std::endl;

        curr_score = icp.getFitnessScore();
		if( score > curr_score )
		{
            score = curr_score; 
			
			pcl::transformPointCloud (*scene, *cloud_out, icp.getFinalTransformation());

            transform1 = icp.getFinalTransformation();
            transform2 = curr_transform;
            
			*t_cloud = *transformed_cloud;
			transformed_cloud.reset();
		}
	}

	for( int i=1; i<=delta; i++ )
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		transformCloud(M_PI/delta*i, 'x', 0, 0, 0, model, transformed_cloud);
        curr_transform = Eigen::AngleAxisf (M_PI/delta*i, Eigen::Vector3f::UnitX());
        
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		icp.setInputSource(scene);
		icp.setInputTarget(transformed_cloud);
		icp.setUseReciprocalCorrespondences(reciprocal);
		pcl::PointCloud<pcl::PointXYZRGB> Final;

		icp.align(Final);
		//std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
		//std::cout << icp.getFinalTransformation() << std::endl;

        curr_score = icp.getFitnessScore();
		if( score > curr_score )
		{
            score = curr_score; 
			
			pcl::transformPointCloud (*scene, *cloud_out, icp.getFinalTransformation());
            
            transform1 = icp.getFinalTransformation();
            transform2 = curr_transform;
            
			*t_cloud = *transformed_cloud;
			transformed_cloud.reset();
		}
	}


   // pcl::io::savePCDFileASCII ("/home/motion/ece590-s2015/perception/single_utils/src/t_model1.pcd", *t_cloud);
   // pcl::io::savePCDFileASCII ("/home/motion/ece590-s2015/perception/single_utils/src/t_scene1.pcd", *cloud_out);
    
	cloud_out.reset();
	t_cloud.reset();
    
    Matrix3f rotation_icp = 
    
    ofstream final_transform;
    final_transform.open (xform_file_name); 
    final_transform << transform;
    final_transform.close();
    
    
	return score;

}

void selectInitialPose( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, Matrix3f &transform, double &translate )
{
    /*Selection of the initial pose for icp. Matching the height
      of object in the scene with one of the dimensions of the model
      is used. Also, the point cloud in the scene is translated so that
      it coincides roughly with the surface of the model. 
    */
    
    // calculating min and max x,y,z coordinates of both model and scene   
    pcl::PointXYZRGB min_model, max_model;
    pcl::getMinMax3D(*model, min_model, max_model );
    double deltax_model = abs(min_model.x-max_model.x);
    double deltay_model = abs(min_model.y-max_model.y);
    double deltaz_model = abs(min_model.z-max_model.z);
    
    //cout<<"Model: "<<deltax_model<<" "<<deltay_model<<" "<<deltaz_model<<endl;
    
    
    pcl::PointXYZRGB min_scene, max_scene;
    pcl::getMinMax3D(*scene, min_scene, max_scene );
    double deltax_scene = abs(min_scene.x-max_scene.x);
    double deltay_scene = abs(min_scene.y-max_scene.y);
    double deltaz_scene = abs(min_scene.z-max_scene.z);
    
    //cout<<"Scene: "<<deltax_scene<<" "<<deltay_scene<<" "<<deltaz_scene<<endl;
    
    /* Let us assume for now that y is the vertical direction. 
       This is used to select an initial pose because it is
       easy to get the length of the object in that direction 
       if the camera is in front of the object. We then match 
       this height with various dimensions of the model and 
       based on that try to turn the model in the general direction
       of the object in the scene.
    */
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    
    *transformed_model = *model;
    if( abs(deltay_scene - deltay_model) > abs(deltay_scene - deltax_model) )
        if( abs(deltay_scene - deltax_model) > abs(deltay_scene - deltaz_model) )
        {
            // x in the model and y in the scene are likely to correspond to one
            // another 
            transform = AngleAxisf(Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitZ()));
            transformCloud(M_PI/2, 'z', 0, 0, 0, model, transformed_model);
            
        }
        else
        {
            // z in the model and y in the scene are likely to correspond to one
            // another 
            transform = AngleAxisf(Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitX()));
            transformCloud(M_PI/2, 'x', 0, 0, 0, model, transformed_model);
        } 
     else if( abs(deltay_scene - deltay_model) > abs(deltay_scene - deltaz_model) )
     {
        // z in the model and y in the scene are likely to correspond to one
        // another 
        transform = AngleAxisf(Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitX()));    
        transformCloud(M_PI/2, 'x', 0, 0, 0, model, transformed_model);
     }
        
     
   //  pcl::io::savePCDFileASCII ("/home/motion/ece590-s2015/perception/single_utils/src/transformed_model.pcd", *transformed_model);
     
     //Roughly aligning the surfaces of the partial object in the scene with the surface of the model.
     //Basically translating the model to some location close to the scene in a way that the exterior 
     //of the scene is in the roughly same direction as exterior of the model. By observing the visualization,
     //it seems like translating the minimum z of the model to coincide with minimum of the scene might do the 
     //trick.
     
     pcl::PointXYZRGB min_t_model, max_t_model;
     pcl::getMinMax3D( *transformed_model, min_t_model, max_t_model );
     
     translate = min_scene.z - min_t_model.z + 0.05;
     
     transformCloud(0, 'x', 0, 0, translate_z, transformed_model, transformed_model);
     
   //  pcl::io::savePCDFileASCII ("/home/motion/ece590-s2015/perception/single_utils/src/transformed_model1.pcd", *transformed_model);
     
     *model = *transformed_model;
     
     transformed_model.reset();
}

int main(int argc, char *argv[])
{

    string model_file;
    string object_file;
    
    
    if (argc > 2)
    {
        model_file = std::string(argv[1]);
        object_file = std::string(argv[2]);
    }
    else
    {
	   return -1;
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    int offset = 0;
 
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFilePLY(model_file, mesh);
    
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud_in);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRS (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (object_file, *cloudRS) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read the object file \n");
      	return (-1);
    }
     
    Matrix3f transform_i; 
    double translate_i;
    selectInitialPose(cloud_in, cloudRS, transform_i, translate_i);
 
	Eigen::Matrix4f transform;

	double score = icp_ext( cloudRS, cloud_in, transform, argv[4] );
    
    // Writing transformed cloud into a file.
    std::cout << transform << "\n";
    if( argc > 3)
    { 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        pcl::transformPointCloud (*cloud_in, *cloud_out, transform);
        
        pcl::io::savePCDFileASCII (argv[3], *cloud_out);
        
        cloud_out.reset();
    }
    
	cloud_in.reset();
	cloudRS.reset();

	return 0;
}


