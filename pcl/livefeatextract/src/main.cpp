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
	SceneSnap = Live.GetCurrentCloud();
	std::vector<int > indices;
	pcl::removeNaNFromPointCloud(*SceneSnap, *SceneSnap, indices);
	cout << "NaNs found at " << indices.size() << " locations." << endl;

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

	for(int i = 0; i < NumSegments; ++i)
	{
		Segmenter->GetSegments().at(i)->ComputeKeypoints();
		if(Segmenter->GetSegments().at(i)->GetNumKeypoints() > 1)
		{
			FeatDescriptor * spin = new FeatDescriptor(std::string("features_live.pcd.txt"), std::string(argv[1]));
			spin->Compute(Segmenter->GetSegments().at(i)->GetCloud(), Segmenter->GetSegments().at(i)->GetKeypointIndices(), i);
			delete spin;
		}
	}

	FileStr.close();

	if(true)
	{
		pcl::visualization::PCLVisualizer viewer("Segment Cloud Viewer");
		for(int i = 0; i < 10; ++i)
		{
//			pcl::visualization::PointCloudGeometryHandlerXYZ<PointXYZRGB > geometry(Segmenter->GetSegments().at(i)->GetCloud());
			string seg = boost::lexical_cast<string>(i);
			viewer.addPointCloud(Segmenter->GetSegments().at(i)->GetCloud(), seg);
//			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, string(i));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, seg);
		}
		viewer.spin();

		while(!viewer.wasStopped()) {}
	}


	return 0;
}

