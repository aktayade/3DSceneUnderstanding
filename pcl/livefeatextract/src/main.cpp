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
	if(argc != 3)
	{
		cerr << "Incorrect number of input parameters. Usage: ./livefeatextract <Config File>" << endl;
		return -1;
	}

	FeatOpenNIGrabber Live;
	Live.Run();

	return -2; // TEMP only

	PointCloud<PointXYZRGBCamSL >::Ptr SceneSnap(new PointCloud<PointXYZRGBCamSL >);

	FeatSegment * Segmenter = new FeatSegment(std::string(argv[2]));
	Segmenter->SetSceneCloud(SceneSnap);
	Segmenter->BreakIntoSegments();

	cout << "Computing keypoints and features for segment..." << endl;
	int NumSegments = Segmenter->GetSegments().size();
	FileStr.open("live_segments.txt", ios::trunc);
	FileStr.close();

	for(int i = 0; i < NumSegments; ++i)
	{
		Segmenter->GetSegments().at(i)->ComputeKeypoints();
		if(Segmenter->GetSegments().at(i)->GetNumKeypoints() > 1)
		{
			FeatDescriptor * spin = new FeatDescriptor(std::string("features_live.pcd.txt"), std::string(argv[2]));
			spin->Compute(Segmenter->GetSegments().at(i)->GetCloud(), Segmenter->GetSegments().at(i)->GetKeypointIndices());
			delete spin;
		}
	}

	return 0;
}

