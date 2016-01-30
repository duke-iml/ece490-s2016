/*
 * convert2pcl.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: bl109
 */

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <string>
#include <iostream>

using namespace std;

typedef map<const string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PointCloudMap;
typedef PointCloudMap::iterator it_PCLMap;

bool readAllPointClouds()
{
	return true;
}

class Detector3D{

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

int main()
{
	const string filename = "/home/motion/ece590-s2015/klampt_models/items/kong_duck_dog_toy/textured_meshes/optimized_tsdf_textured_mesh.ply";
	pcl::PLYReader plyReader;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    int offset = 0;

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

	plyReader.read(filename,*cloud_in, offset);



	*cloud_out=*cloud_in;
	for (size_t i = 0; i < cloud_in->points.size (); ++i)
	{
	    cloud_out->points[i].x = cloud_in->points[i].x + 0.2f;
	    cloud_out->points[i].y = cloud_in->points[i].y + 0.3f;
	}

	//viewer.showCloud(cloud_in);
	viewer.showCloud(cloud_out);
	while (!viewer.wasStopped ())
	{
	}

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

	icp.setInputSource(cloud_out);
	icp.setInputTarget(cloud_in);
	pcl::PointCloud<pcl::PointXYZRGB> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;



	/*
    Eigen::Matrix4f outT;
    double score;


    string dir = "/home/bl109/Documents/ece590-s2015-ece590-s2015/klampt_models/items/";
    Detector3D detector(const_cast<char*>(dir.c_str()));
    detector.locateObject(cloud_out, "kong_duck_dog_toy", outT, score);
    cout<<outT<<endl;
    cout<<"Score: "<<score<<endl;*/

	cloud_in.reset();
	cloud_out.reset();

	return 0;
}


