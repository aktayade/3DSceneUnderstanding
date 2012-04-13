#include "FeatPointCloud.hpp"

FeatPointCloud::FeatPointCloud(void)
{
	std::cout << "Please use the other constructor. This is empty." << std::endl;
}

FeatPointCloud::FeatPointCloud(const std::string& CloudFile) : 
	m_Cloud(new PointCloud<PointXYZRGB >),
	m_KeypointDetector(new FeatKeypoint)
{
	if(pcl::io::loadPCDFile<PointXYZRGB>(CloudFile.c_str(), *m_Cloud) == -1)
		std::cerr << "Problem loading from file " << CloudFile << ". Nothing stored in FeatPointCloud." << std::endl;
	else
		std::cout << "Successfully loaded " << CloudFile << "." << std::endl;
}

FeatPointCloud::FeatPointCloud(PointCloud<PointXYZRGBCamSL >::Ptr Cloud, std::vector<int > CopyIndices) :
	m_Cloud(new PointCloud<PointXYZRGB >),
	m_KeypointDetector(new FeatKeypoint)
{
	pcl::copyPointCloud(*Cloud, CopyIndices, *m_Cloud);
}

FeatPointCloud::~FeatPointCloud()
{

}

bool FeatPointCloud::ComputeKeypoints(void)
{
	if(m_Cloud)
	{
		if(m_KeypointDetector->Compute(m_Cloud))
		{
			SnapKeypoints2Cloud();
			return true;
		}
		return false;
	}

	std::cout << "FeatPointCloud not initialized. Check usage." << std::endl;
	return false;
}

void FeatPointCloud::SnapKeypoints2Cloud(void)
{
	for(int i = 0; i < m_KeypointDetector->GetKeypoints()->points.size(); ++i)
	{
		std::vector<int > PtIdx(1);
		std::vector<float > PtDist;

		m_KNNKdTree.setInputCloud(m_Cloud);
      
		pcl::PointCloud<PointXYZRGB>::Ptr siftcloud_XYZRGB(new pcl::PointCloud<PointXYZRGB >);
		pcl::copyPointCloud(*m_KeypointDetector->GetKeypoints(), *siftcloud_XYZRGB);

		m_KNNKdTree.nearestKSearch(*siftcloud_XYZRGB, i, 1, PtIdx, PtDist);
		m_KeypointIndices.push_back(PtIdx.at(0));
	}
	
	m_NumKeypoints = m_KeypointIndices.size();

//	std::cout << "Done snapping keypoints to point cloud." << std::endl;
}

std::vector<int > FeatPointCloud::GetKeypointIndices(void)
{
	if(m_KeypointIndices.size() < 1)
		std::cerr << "Keypoint indices not generated. No indices stored." << std::endl;

	return m_KeypointIndices;
}

