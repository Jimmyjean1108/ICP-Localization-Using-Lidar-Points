#include <iostream>
#include <string>
#include <sstream>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//Downsample the pointcloud
pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
std::string last_map;
int map_select_trigger = 0;

std::string* map_select(float x, float y){
	//Find the central coordinate
	int x_ = int(x) - int(x)%100;
	int y_ = int(y) - int(y)%100;
	//Find the nine closest maps
	int map_x[3] = {x_ - 100, x_, x_ + 100};
	int map_y[3] = {y_ - 100, y_, y_ + 100};

	//Write the map name to the string array
	std::stringstream map;
	static std::string map_name[9];
	int k = 0;
	for (int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			map << "map_" << map_x[i] << "_" << map_y[j] <<".pcd";
			map_name[k] = map.str();
			k++;
			map.str("");
			map.clear();
		}
	}
	return map_name;
}

pcl::PCLPointCloud2::Ptr merge_maps(const std::string* map_name){

	if(last_map == map_name[4] || map_select_trigger > 0){
		map_select_trigger--;
		return cloud_filtered;
	}
	else{
		std::cerr << "Start merging maps." << std::endl;
		pcl::PCDReader reader;
		//Read a single map
		pcl::PCLPointCloud2::Ptr cloud_unit (new pcl::PCLPointCloud2 ());
		//Merge to a big map
		pcl::PCLPointCloud2::Ptr cloud_combined (new pcl::PCLPointCloud2 ());
		
		std::string file_name;
		
		for(int i=0;i<9;i++){
			file_name = "/home/jimmy/catkin_ws/src/localization_competition/test/nuscenes_maps/" + map_name[i];
			if(reader.read (file_name, *cloud_unit) == 0){
				//*cloud_combined +=  *cloud_unit;
				std::cerr << i << " ";
				//std::cerr << cloud_unit->width * cloud_unit->height << ",";
				pcl::concatenatePointCloud (*cloud_combined, *cloud_unit, *cloud_combined);
				//std::cerr << cloud_combined->width * cloud_combined->height << std::endl;
			}
		}
		last_map = map_name[4];
		map_select_trigger = 5;

		sor.setInputCloud (cloud_combined);
		sor.setLeafSize (1.0f, 1.0f, 1.0f);
		sor.filter (*cloud_filtered);

		return cloud_filtered;
	}	
}

