#include <iostream>

#include <pcl/visualization/cloud_viewer.h>

#include "FeatPointCloud.hpp"
#include "FeatKeypoint.hpp"
#include "FeatDescriptor.hpp"

using namespace std;
using namespace pcl;

int main(int argc, char ** argv)
{
	if(argc != 2)
	{
		cerr << "Incorrect number of input parameters. Usage: ./featextract <Input PCD File>" << endl;
		return -1;
	}
	
	// Load data to a FeatPointCloud object
	FeatPointCloud * PCData = new FeatPointCloud(argv[1]);
	// Compute keypoints for PCData
	PCData->ComputeKeypoints();

	// Compute features
	FeatDescriptor * m_Spin = new FeatDescriptor();
	m_Spin->Compute(PCData->GetCloud(), PCData->GetKeypointIndices());

	// Visualize stuff
	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	pcl::visualization::PointCloudGeometryHandlerXYZ< PointWithScale > geometry(PCData->GetKeypointDetector()->GetKeypoints());
	viewer.addPointCloud(PCData->GetCloud(), "cloud");
	viewer.addPointCloud(PCData->GetKeypointDetector()->GetKeypoints(), geometry, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "keypoints");
	viewer.spin();

	while(!viewer.wasStopped()) {}

	return 0;
}

