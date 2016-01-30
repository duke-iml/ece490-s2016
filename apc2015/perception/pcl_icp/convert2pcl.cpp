/*
 * convert2pcl.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: bl109
 */
 
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

bool readAllPointClouds()
{
	return true;
}

/*class Detector3D{

	private:

		PointCloudMap pointCloudMap;
		std::string object_names[25];


	public:
		Detector3D(char* dir){

			char filename[500];

			//fill object_names with the names of the objects
			object_names[0]="oreo_mega_stuf";
			object_names[1]="champion_copper_plus_spark_plug";
			object_names[2]="expo_dry_erase_board_eraser";
			object_names[3]="kong_duck_dog_toy";
			object_names[4]="genuine_joe_plastic_stir_sticks";
			object_names[5]="munchkin_white_hot_duck_bath_toy";
			object_names[6]="crayola_64_ct";
			object_names[7]="mommys_helper_outlet_plugs";
			object_names[8]="sharpie_accent_tank_style_highlighters";
			object_names[9]="kong_air_dog_squeakair_tennis_ball";
			object_names[10]="stanley_66_052";
			object_names[11]="safety_works_safety_glasses";
			object_names[12]="dove_beauty_bar";
			object_names[13]="one_with_nature_soap_dead_sea_mud";
			object_names[14]="cheezit_bit_original";
			object_names[15]="paper_mate_12_count_mirado_black_warrior";
			object_names[16]="feline_greenies_dental_treats";
			object_names[17]="elmers_washable_no_run_school_glue";
			object_names[18]="mead_index_cards";
			object_names[19]="rollodex_mesh_collection_jumbo_pencil_cup";
			object_names[20]="first_years_take_and_toss_straw_cups";
			object_names[21]="highland_6539_self_stick_notes";
			object_names[22]="mark_twain_huckleberry_finn";
			object_names[23]="kygen_squeakin_eggs_plush_puppies";
			object_names[24]="kong_sitting_frog_dog_toy";


			pcl::PLYReader plyReader;

			// fill the pointCloudMap by all candidate objects
			for(int i=0; i<25; i++)
			{
				char key[500];

				strcpy(key, dir);
				strcat(key, object_names[i].c_str());

				strcpy(filename, key);
				strcat(filename, "/textured_meshes/optimized_tsdf_textured_mesh.ply");

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

				plyReader.read(filename, *cloud, 0);


				this->pointCloudMap.insert(std::pair<const string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(object_names[i],cloud));

				filename[0]=0;

			}

		}

		~Detector3D(){

			//freeing the pointers

			for(it_PCLMap it = pointCloudMap.begin(); it != pointCloudMap.end(); it++)
			    it->second.reset();

		}

		double locateObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inPointCloud, const string objectName, Eigen::Matrix4f& outT, double& score)
		{
			it_PCLMap it = this->pointCloudMap.find(objectName);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr outPointCloud;

			it = this->pointCloudMap.find(objectName);

			if( it == this->pointCloudMap.end() )
			{
				cout<<"No object with that name found.!"<<endl;
				return -1;
			}

			outPointCloud = it->second;

			pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
			icp.setInputSource(inPointCloud);
			icp.setInputTarget(outPointCloud);
			pcl::PointCloud<pcl::PointXYZRGB> Final;
			icp.align(Final);

			outT = icp.getFinalTransformation();

			score = icp.getFitnessScore();

			return icp.hasConverged();
		}

};
*/

double icp_ext( pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, Eigen::Matrix4f & transform )
{
	double curr_score, score;
	Eigen::Matrix4f curr_transform;
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

	transform = icp.getFinalTransformation();

    int delta = 8;

	for( int i=1; i<=delta; i++ )
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		transformCloud(M_PI/delta*i, 'x', 0, 0, 0, model, transformed_cloud);

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
			transform = icp.getFinalTransformation()*curr_transform;
			
			*t_cloud = *transformed_cloud;
			transformed_cloud.reset();
		}
	
	}

	for( int i=1; i<=delta; i++ )
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		transformCloud(M_PI/delta*i, 'x', 0, 0, 0, model, transformed_cloud);

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
			transform = curr_transform;
			pcl::transformPointCloud (*scene, *cloud_out, icp.getFinalTransformation());

            transform = icp.getFinalTransformation()*curr_transform;
			*t_cloud = *transformed_cloud;
			transformed_cloud.reset();
		}
	}

	for( int i=1; i<=delta; i++ )
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		transformCloud(M_PI/delta*i, 'x', 0, 0, 0, model, transformed_cloud);

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
            transform = icp.getFinalTransformation()*curr_transform;
			*t_cloud = *transformed_cloud;
			transformed_cloud.reset();
		}
	}

	/*pcl::visualization::CloudViewer viewer ("Transformed model");
	viewer.showCloud(t_cloud);
	while (!viewer.wasStopped ())
	{
	}

	pcl::visualization::CloudViewer viewer1 ("Cloud_out");
	viewer1.showCloud(cloud_out);
	while (!viewer1.wasStopped ())
	{
	}*/

   // pcl::io::savePCDFileASCII ("/home/motion/ece590-s2015/perception/single_utils/src/t_model1.pcd", *t_cloud);
   // pcl::io::savePCDFileASCII ("/home/motion/ece590-s2015/perception/single_utils/src/t_scene1.pcd", *cloud_out);
    
	cloud_out.reset();
	t_cloud.reset();
    
    ofstream final_transform;
    final_transform.open ("/home/motion/ece590-s2015/perception/single_utils/xform_matrix.txt");
    final_transform << transform;
    final_transform.close();
    
	return score;

}

void selectInitialPose( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene )
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
            
            transformCloud(M_PI/2, 'z', 0, 0, 0, model, transformed_model);
        }
        else
        {
            // z in the model and y in the scene are likely to correspond to one
            // another 
            
            transformCloud(M_PI/2, 'x', 0, 0, 0, model, transformed_model);
        } 
     else if( abs(deltay_scene - deltay_model) > abs(deltay_scene - deltaz_model) )
     {
        // z in the model and y in the scene are likely to correspond to one
        // another 
            
        transformCloud(M_PI/2, 'x', 0, 0, 0, model, transformed_model);
     }
        
     
     pcl::io::savePCDFileASCII ("/home/motion/ece590-s2015/perception/single_utils/src/transformed_model.pcd", *transformed_model);
     /*pcl::io::savePCDFileASCII ("/home/motion/ece590-s2015/perception/single_utils/src/scene.pcd", *scene);
     
     pcl::io::savePCDFileASCII ("/home/motion/ece590-s2015/perception/single_utils/src/model.pcd", *model);*/
       
     //Roughly aligning the surfaces of the partial object in the scene with the surface of the model.
     //Basically translating the model to some location close to the scene in a way that the exterior 
     //of the scene is in the roughly same direction as exterior of the model. By observing the visualization,
     //it seems like translating the minimum z of the model to coincide with minimum of the scene might do the 
     //trick.
     
     pcl::PointXYZRGB min_t_model, max_t_model;
     pcl::getMinMax3D( *transformed_model, min_t_model, max_t_model );
     
     transformCloud(0, 'x', 0, 0, min_scene.z - min_t_model.z + 0.05,transformed_model, transformed_model);
     
     pcl::io::savePCDFileASCII ("/home/motion/ece590-s2015/perception/single_utils/src/transformed_model1.pcd", *transformed_model);
     
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
	   model_file = "genuine_joe.ply";
	   object_file = "genuine_joe_scene.pcd";
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    int offset = 0;
 
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFilePLY(model_file, mesh);
    
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud_in);

	/*pcl::visualization::CloudViewer viewer3 ("Simple Cloud Viewer");	
	viewer3.showCloud(cloud_in);
	while (!viewer3.wasStopped ())
	{
	}*/

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRS (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (object_file, *cloudRS) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read the object file \n");
      	return (-1);
    }
     
    pcl::visualization::CloudViewer viewer1 ("Simple Cloud Viewer");
	
	viewer1.showCloud(cloudRS);
	while (!viewer1.wasStopped ())
	{
	}
        
    selectInitialPose(cloud_in, cloudRS);
 
	Eigen::Matrix4f transform;

	double score = icp_ext( cloudRS, cloud_in, transform );
    
    // Writing transformed cloud into a file.
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


