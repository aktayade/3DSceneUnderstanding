#include <iostream>

#include <pcl/visualization/cloud_viewer.h>

#include "FeatPointCloud.hpp"
#include "FeatKeypoint.hpp"
#include "FeatDescriptor.hpp"
#include "FeatSegment.hpp"
#include "extern.hpp"


using namespace std;
using namespace pcl;

int main(int argc, char ** argv)
{
	if(argc != 2)
	{
		cerr << "Incorrect number of input parameters. Usage: ./featextract <Input PCD File>" << endl;
		return -1;
	}
	

	PointCloud<PointXYZRGBCamSL >::Ptr WholeScene(new PointCloud<PointXYZRGBCamSL >);
	// Load data into a PointXYZRGBCamSL
	if(pcl::io::loadPCDFile(argv[1], *WholeScene) == -1)
		std::cerr << "Problem loading from file " << argv[1] << "." << std::endl;
	else
		std::cout << "Successfully loaded " << argv[1] << "." << std::endl;

	// Split scene into segments
	FeatSegment * Segmenter = new FeatSegment;
	Segmenter->SetSceneCloud(WholeScene);
	Segmenter->BreakIntoSegments();

	vector<FeatDescriptor * > SpinImages;
	cout << "Computing keypoints and features for segment..." << endl;
	int NumSegments = Segmenter->GetSegments().size();
	for(int i = 0; i < NumSegments; ++i)
	{
//		cout << "Segment " << i+1 << "\t";
		cout << i+1 << endl;
		Segmenter->GetSegments().at(i)->ComputeKeypoints();

		// Compute features
		if(Segmenter->GetSegments().at(i)->GetNumKeypoints() > 0)
		{
			FeatDescriptor * spin = new FeatDescriptor();
			spin->Compute(Segmenter->GetSegments().at(i)->GetCloud(), Segmenter->GetSegments().at(i)->GetKeypointIndices());
			SpinImages.push_back(spin);
		}
	}

//	// Visualize stuff
//	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
//	pcl::visualization::PointCloudGeometryHandlerXYZ< PointWithScale > geometry(PCData->GetKeypointDetector()->GetKeypoints());
//	viewer.addPointCloud(PCData->GetCloud(), "cloud");
//	viewer.addPointCloud(PCData->GetKeypointDetector()->GetKeypoints(), geometry, "keypoints");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "keypoints");
//	viewer.spin();

//	while(!viewer.wasStopped()) {}

	return 0;
}

