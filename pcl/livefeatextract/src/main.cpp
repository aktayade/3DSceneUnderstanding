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
		cerr << "Incorrect number of input parameters. Usage: ./livefeatextract <Output Feature File> <Config File>" << endl;
		return -1;
	}

	FeatOpenNIGrabber Live;
	Live.Run();

	return -2; // TEMP only

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

	cout << "Computing keypoints and features for segment..." << endl;
	int NumSegments = Segmenter->GetSegments().size();
	for(int i = 0; i < NumSegments; ++i)
	{
		Segmenter->GetSegments().at(i)->ComputeKeypoints();
		cout << i << " has num keypoints as " << Segmenter->GetSegments().at(i)->GetKeypointDetector()->GetKeypoints()->points.size() << endl;

		// Compute features
		if(Segmenter->GetSegments().at(i)->GetNumKeypoints() > 1) // PARAM - Also depends on SPINMinPts
		{
			FeatDescriptor * spin = new FeatDescriptor(std::string(argv[2]), std::string(argv[3]));
			spin->Compute(Segmenter->GetSegments().at(i)->GetCloud(), Segmenter->GetSegments().at(i)->GetKeypointIndices());
			delete spin;
		}
	}

	return 0;
}

