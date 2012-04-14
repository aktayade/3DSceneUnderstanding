#ifndef __FEATKEYPOINT_H__
#define __FEATKEYPOINT_H__

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fstream>

using namespace pcl;

class FeatKeypoint
{
	public:
		FeatKeypoint(const std::string& ConfigFName);
		virtual ~FeatKeypoint();

		bool Compute(PointCloud<PointXYZRGB >::Ptr Cloud);
		PointCloud<PointWithScale >::Ptr GetKeypoints(void) { return m_Keypoints; };

	private:
		bool ParseConfig(void);
		void SetParams(void);

		PointCloud<PointWithScale >::Ptr m_Keypoints;
		pcl::search::KdTree<PointXYZRGB >::Ptr m_FeatCloudKdTree;
	
		SIFTKeypoint<PointXYZRGB, PointWithScale > m_SIFT;

		//! SIFT keypoint parameters
		std::bitset<4 > m_isAllSet;
		std::string m_ConfigFName;
		float m_SIFTMinScale;
		int m_SIFTOctaves;
		int m_SIFTScalesPerOctave;
		float m_SIFTMinContrast;
};

#endif /* __FEATKEYPOINT_H__ */

