#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "extern.hpp"

using namespace std;
using namespace pcl;

vector<vector<int > > SegIndices;

bool ParseSegments(void)
{
	ifstream FileStr("live_segments.txt");
	cout << "Start" << endl;
	if(FileStr.is_open())
	{
		while(FileStr.good())
		{
			string line, word;
			getline(FileStr, line);
			stringstream ss(line);
			ss >> word;
			vector<int > tmp;
			while(ss.good())
				tmp.push_back(atoi(ss.str().c_str()));
	
			SegIndices.push_back(tmp);
		}
	}
	else
		return false;

	FileStr.close();
	cout << "Done" << endl;
	return true;
}

bool ParseLabels(void)
{

}

int main(int argc, char** argv)
{
	PointCloud<PointXYZRGB >::Ptr cloud(new PointCloud<PointXYZRGB >);

	if(pcl::io::loadPCDFile<PointXYZRGB >("live.pcd", *cloud) == -1 || ParseSegments() == false || ParseLabels() == false)
	{
		cout << "Couldn't read some files. Must be run in sequence." << endl;
		return -1;
	}	

	pcl::visualization::PCLVisualizer viewer("Classified Output");
//	viewer.setFullScreen(true); // CAREFUL!
	pcl::visualization::PointCloudGeometryHandlerXYZ<PointXYZRGB > geometry(cloud);
	viewer.addPointCloud(cloud, geometry, "cloud");
//	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "keypoints");
	viewer.spin();

	while(!viewer.wasStopped())
	{

	}

	return 0;
}

