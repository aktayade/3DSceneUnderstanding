#include <iostream>

#include "FeatPointCloud.hpp"
#include "FeatKeypoint.hpp"
#include "FeatDescriptor.hpp"
#include "FeatSegment.hpp"
#include "extern.hpp"
#include "FeatOpenNIGrabber.hpp"

using namespace std;
using namespace pcl;

int main(int argc, char ** argv)
{
	if(argc != 2)
	{
		cerr << "Incorrect number of input parameters. Usage: ./livefeatextract <Config File>" << endl;
		return -1;
	}

	FeatOpenNIGrabber Live;
	Live.Run();

	PointCloud<PointXYZRGB >::Ptr SceneSnap(new PointCloud<PointXYZRGB >);
	copyPointCloud<PointXYZRGB, PointXYZRGB >(*Live.GetCurrentCloud(), *SceneSnap);
	std::vector<int > indices;
	pcl::removeNaNFromPointCloud(*Live.GetCurrentCloud(), *SceneSnap, indices);
//	cout << "NaNs found at " << indices.size() << " locations." << endl;

	FeatSegment * Segmenter = new FeatSegment(std::string(argv[1]));
	Segmenter->SetSceneCloud(SceneSnap);
	Segmenter->BreakIntoSegments();

	cout << "Computing keypoints and features for segment..." << endl;
	int NumSegments = Segmenter->GetSegments().size();
//	FileStr2.open("live_segments.txt", ios::trunc);
//	FileStr2.close();
	// Open new file ot just clear the existing file
	FileStr.open("features_live.pcd.txt", ios::trunc);
	if(FileStr.is_open())
		FileStr.close();

	FileStr.open("features_live.pcd.txt", ios::app);
	FileStr << "#SegmentNumber #SpaceSeparatedSpinImage (153)" << endl;

//	for(int i = 0; i < NumSegments; ++i)
//	{
//		Segmenter->GetSegments().at(i)->ComputeKeypoints();
//		if(Segmenter->GetSegments().at(i)->GetNumKeypoints() > 1)
//		{
//			FeatDescriptor * spin = new FeatDescriptor(std::string("features_live.pcd.txt"), std::string(argv[1]));
//			spin->Compute(Segmenter->GetSegments().at(i)->GetCloud(), Segmenter->GetSegments().at(i)->GetKeypointIndices(), i);
//			delete spin;
//		}
//	}

	FileStr.close();
	sleep(1);

	int SegNum = 100;
	if(true)
	{
		pcl::visualization::PCLVisualizer viewer("Segment Cloud Viewer");
		viewer.addPointCloud(Segmenter->GetSegments().at(SegNum)->GetCloud(), "cloud");
//		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
//		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "keypoints");
		viewer.spin();

		while(!viewer.wasStopped()) {}
	}


	return 0;
}

