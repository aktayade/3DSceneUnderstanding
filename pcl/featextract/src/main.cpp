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
	if(argc < 4)
	{
		cerr << "Incorrect number of input parameters. Usage: ./featextract <Input PCD File> <Output Feature File> <Config File> [<Segment number to visualize>]" << endl;
		return -1;
	}
	// Create file if does not exist. Open it if it does
	FileStr.open(argv[2], fstream::in | fstream::out | fstream::app);

	PointCloud<PointXYZRGBCamSL >::Ptr WholeScene(new PointCloud<PointXYZRGBCamSL >);
	// Load data into a PointXYZRGBCamSL
	if(pcl::io::loadPCDFile(argv[1], *WholeScene) == -1)
		std::cerr << "Problem loading from file " << argv[1] << "." << std::endl;
	else
		std::cout << "Successfully loaded " << argv[1] << "." << std::endl;

	// Split scene into segments
	FeatSegment * Segmenter = new FeatSegment(std::string(argv[3]));
	Segmenter->SetSceneCloud(WholeScene);
	Segmenter->BreakIntoSegments();

	vector<FeatDescriptor * > SpinImages;
	cout << "Computing keypoints and features for segment..." << endl;
	int NumSegments = Segmenter->GetSegments().size();
	for(int i = 0; i < NumSegments; ++i)
//	for(int i = 221; i < 222; ++i)
	{
//		cout << i+1 << endl;
		Segmenter->GetSegments().at(i)->ComputeKeypoints();
		cout << i << " has num keypoints as " << Segmenter->GetSegments().at(i)->GetKeypointDetector()->GetKeypoints()->points.size() << endl;


		// Compute features
		if(Segmenter->GetSegments().at(i)->GetNumKeypoints() > 5) // PARAM - minimum number of keypoints
		{
			FeatDescriptor * spin = new FeatDescriptor(std::string(argv[2]), std::string(argv[3]));
			FileStr << Segmenter->GetLabels().at(i) << std::endl;
			
			spin->Compute(Segmenter->GetSegments().at(i)->GetCloud(), Segmenter->GetSegments().at(i)->GetKeypointIndices());
			SpinImages.push_back(spin);
		}
	}

	FileStr.close();

	// Visualize stuff
	int SegNum;
	if(!argv[4])
		SegNum = 401;
	else
		SegNum = atoi(argv[4]);
	
	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	pcl::visualization::PointCloudGeometryHandlerXYZ<PointXYZRGB > geometry(Segmenter->GetSegments().at(SegNum)->GetKeypointDetector()->GetKeypoints());
	viewer.addPointCloud(Segmenter->GetSegments().at(SegNum)->GetCloud(), "cloud");
	viewer.addPointCloud(Segmenter->GetSegments().at(SegNum)->GetKeypointDetector()->GetKeypoints(), geometry, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "keypoints");
	viewer.spin();

	while(!viewer.wasStopped()) {}	


	return 0;
}


