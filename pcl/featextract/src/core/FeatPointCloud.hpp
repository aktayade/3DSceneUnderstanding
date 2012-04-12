#ifndef __FEATPOINTCLOUD_H__
#define __FEATPOINTCLOUD_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "FeatKeypoint.hpp"
#include "extern.hpp"

using namespace pcl;

class FeatPointCloud
{
	public:
		FeatPointCloud(void);
		FeatPointCloud(const std::string& CloudFile);
		FeatPointCloud(PointCloud<PointXYZRGBCamSL >::Ptr Cloud, std::vector<int > CopyIndices);
		virtual ~FeatPointCloud(void);

		PointCloud<PointXYZRGB >::Ptr GetCloud(void) { return m_Cloud; };
		FeatKeypoint * GetKeypointDetector(void) { return m_KeypointDetector; };
		std::vector<int > GetKeypointIndices(void);
		int GetNumKeypoints(void) { return m_NumKeypoints; }
		void SetCloud(PointCloud<PointXYZRGB >::Ptr Cloud) { m_Cloud = Cloud; };
		
		bool ComputeKeypoints(void);

	private:
		PointCloud<PointXYZRGB >::Ptr m_Cloud;
		FeatKeypoint * m_KeypointDetector;
		std::vector<int > m_KeypointIndices;
		int m_NumKeypoints;
		pcl::KdTreeFLANN<PointXYZRGB > m_KNNKdTree;

		void SnapKeypoints2Cloud(void);
};

#endif /* __FEATPOINTCLOUD_H__ */

