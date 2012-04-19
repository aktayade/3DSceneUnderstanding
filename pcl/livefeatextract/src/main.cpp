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
		cerr << "Incorrect number of input parameters. Usage: ./featextract <Output Feature File> <Config File>" << endl;
		return -1;
	}
	
	// Open new file ot just clear the existing file
	FileStr.open(argv[2], ios::trunc);
	if(FileStr.is_open())
		FileStr.close();

	FileStr.open(argv[2], ios::app);

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

//	vector<FeatDescriptor * > SpinImages;
//	cout << "Computing keypoints and features for segment..." << endl;
//	int NumSegments = Segmenter->GetSegments().size();
//	for(int i = 0; i < NumSegments; ++i)
//	{
//		Segmenter->GetSegments().at(i)->ComputeKeypoints();
//		cout << i << " has num keypoints as " << Segmenter->GetSegments().at(i)->GetKeypointDetector()->GetKeypoints()->points.size() << endl;

//		// Compute features
//		if(Segmenter->GetSegments().at(i)->GetNumKeypoints() > 1) // PARAM - Also depends on SPINMinPts
//		{
//			FeatDescriptor * spin = new FeatDescriptor(std::string(argv[2]), std::string(argv[3]));
//			FileStr << Segmenter->GetLabels().at(i) << std::endl;
//			
//			spin->Compute(Segmenter->GetSegments().at(i)->GetCloud(), Segmenter->GetSegments().at(i)->GetKeypointIndices());
////			SpinImages.push_back(spin);
//			delete spin;
//		}
//	}

	FileStr.close();

	return 0;
}

