#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main (int argc, char** argv)
{
	if(argc != 2)
	{
		cout << "Incorrect number of arguments. Usage: spinextract <Input PCD File>" << endl;
		return -2;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if(pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file \n");
		return -1;
	}

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while(!viewer.wasStopped())
	{

	}

	return 0;
}

