#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/algorithm/string.hpp>


#include "FeatPointCloud.hpp"
#include "FeatKeypoint.hpp"
#include "FeatDescriptor.hpp"
#include "FeatSegment.hpp"
#include "extern.hpp"

using namespace std;
using namespace pcl;

bool isVisualize = false;
bool isUseISFeatures = false;

void hline(const int& Count = 80)
{
	for(int i = 0; i < Count; ++i)
		cout << "=";

	cout << endl;
}
	
int ExtractSceneNumber(const string& FileName)
{
	// Can handle only scene followed by 1 or 2 numbers
	string SceneNum = FileName;
	SceneNum.erase(0, SceneNum.size()-11);
//	cout << SceneNum << endl;
	if(SceneNum[0] != 's')
	{
		SceneNum.erase(0, 6);
		SceneNum.erase(1, SceneNum.size()-1);
//		cout << atoi(SceneNum.c_str()) << endl; 
		return atoi(SceneNum.c_str());
	}

	SceneNum.erase(0, 5);
	SceneNum.erase(2, SceneNum.size()-1);
//	cout << atoi(SceneNum.c_str()) << endl; 
	return atoi(SceneNum.c_str());
}

int main(int argc, char ** argv)
{
	if(argc < 4)
	{
		cerr << "Incorrect number of input parameters. Usage: ./featextract <Input PCD File> <Output Feature File> <Config File> [Cornell I+S Features File] [<Visualize? (Type anything)>] [<Segment number to visualize>]" << endl;
		return -1;
	}
	
	if(argv[5])
		isVisualize = true;

	if(argv[4])
		isUseISFeatures = true;

	// Open new file ot just clear the existing file
	FileStr.open(argv[2], ios::trunc);
	if(FileStr.is_open())
		FileStr.close();

	FileStr.open(argv[2], ios::app);

	PointCloud<PointXYZRGBCamSL >::Ptr WholeScene(new PointCloud<PointXYZRGBCamSL >);
	// Load data into a PointXYZRGBCamSL
	if(pcl::io::loadPCDFile(argv[1], *WholeScene) == -1)
	{
		std::cerr << "[FATAL]: Problem loading from file " << argv[1] << "." << std::endl;
		return -2;
	}
	hline();
	cout << "[SUCCESS]: Successfully loaded " << argv[1] << endl;
	hline();

	int SceneNumber = ExtractSceneNumber(string(argv[1]));
	cout << "[OK]: Recognized as scene " << SceneNumber << "." << endl;

	// Split scene into segments
	FeatSegment * Segmenter = new FeatSegment(std::string(argv[3]));
	Segmenter->SetSceneCloud(WholeScene);
	if(isUseISFeatures)
		isUseISFeatures = Segmenter->BreakIntoSegments(std::string(argv[4]), SceneNumber);
	else
		Segmenter->BreakIntoSegments();

	cout << "[INFO]: Computing keypoints and features for segment..." << endl;
	int NumSegments = Segmenter->GetSegments().size();
	for(int i = 0; i < NumSegments; ++i)
	{
		Segmenter->GetSegments().at(i)->ComputeKeypoints();
//		cout << i << " has num keypoints as " << Segmenter->GetSegments().at(i)->GetKeypointDetector()->GetKeypoints()->points.size() << endl;

		// Compute features
		if(Segmenter->GetSegments().at(i)->GetNumKeypoints() > 1) // PARAM - Also depends on SPINMinPts
		{
			FeatDescriptor * spin = new FeatDescriptor(std::string(argv[2]), std::string(argv[3]));
			FileStr << Segmenter->GetLabels().at(i) << std::endl;
			if(isUseISFeatures)
				spin->Compute(Segmenter->GetSegments().at(i)->GetCloud(), Segmenter->GetSegments().at(i)->GetKeypointIndices(), Segmenter->GetISFeatures().at(i), Segmenter->GetLabels().at(i));
			else
				spin->Compute(Segmenter->GetSegments().at(i)->GetCloud(), Segmenter->GetSegments().at(i)->GetKeypointIndices());

			delete spin;
		}
	}
	cout << "[INFO]: Done computing features and keypoints." << endl;
	FileStr.close();

	// Visualization stuff
	int SegNum;
	if(!argv[6])
		SegNum = 210; // PARAM - Just for example. Might overflow.
	else
		SegNum = atoi(argv[6]);

	if(isVisualize)
	{
		pcl::visualization::PCLVisualizer viewer("Segment Cloud Viewer");
		pcl::visualization::PointCloudGeometryHandlerXYZ<PointXYZRGB > geometry(Segmenter->GetSegments().at(SegNum)->GetKeypointDetector()->GetKeypoints());
		viewer.addPointCloud(Segmenter->GetSegments().at(SegNum)->GetCloud(), "cloud");
		viewer.addPointCloud(Segmenter->GetSegments().at(SegNum)->GetKeypointDetector()->GetKeypoints(), geometry, "keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "keypoints");
		viewer.spin();

		while(!viewer.wasStopped()) {}
	}

	return 0;
}

