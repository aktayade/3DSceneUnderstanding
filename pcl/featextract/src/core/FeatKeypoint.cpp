#include "FeatKeypoint.hpp"

FeatKeypoint::FeatKeypoint(void) :
	m_Keypoints(new PointCloud<PointWithScale >),
	m_FeatCloudKdTree(new pcl::search::KdTree<PointXYZRGB >)
{
	// PARAM
	// Constants from http://pastebin.com/8s5k8M1A
	const float min_scale = 0.01;
	const int nr_octaves = 3;
	const int nr_scales_per_octave = 3;
	const float min_contrast = 10.0;

	// Set some default initial parameters
	m_SIFT.setScales(min_scale, nr_octaves, nr_scales_per_octave);
	m_SIFT.setMinimumContrast(min_contrast);
	m_SIFT.setSearchMethod(m_FeatCloudKdTree);
}

FeatKeypoint::~FeatKeypoint(void)
{

}

bool FeatKeypoint::Compute(PointCloud<PointXYZRGB >::Ptr Cloud)
{
	std::cout << Cloud->points.size() << " points in loaded point cloud." << std::endl;
	m_SIFT.setInputCloud(Cloud);
	m_SIFT.compute(*m_Keypoints);
	std::cout << m_Keypoints->points.size() << " keypoints detected successfully." << std::endl;

	return true;
}

