#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "extern.hpp"

using namespace std;
using namespace pcl;

int main (int argc, char** argv)
{
	if(argc != 2)
	{
		cout << "Incorrect number of arguments. Usage: visualize <Input PCD File> [Use label?]" << endl;
		return -2;
	}

	PointCloud<PointXYZRGBCamSL >::Ptr cloud(new PointCloud<PointXYZRGBCamSL >);

	if(pcl::io::loadPCDFile<PointXYZRGBCamSL >(argv[1], *cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file \n");
		return -1;
	}

	pcl::visualization::PCLVisualizer viewer("Classified Output");
	viewer.setFullScreen(true);
	pcl::visualization::PointCloudGeometryHandlerXYZ<PointXYZRGBCamSL > geometry(cloud);
	viewer.addPointCloud(cloud, geometry, "cloud");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "keypoints");
	viewer.spin();

	while(!viewer.wasStopped())
	{

	}

	return 0;
}

