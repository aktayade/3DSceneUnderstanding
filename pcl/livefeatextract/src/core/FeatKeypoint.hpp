#ifndef __FEATKEYPOINT_H__
#define __FEATKEYPOINT_H__

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fstream>

#define M_KEYPOINT_SIFT 0
#define M_KEYPOINT_UNIFORM 1

using namespace pcl;

class FeatKeypoint
{
	public:
		FeatKeypoint(const std::string& ConfigFName);
		virtual ~FeatKeypoint();

		bool Compute(PointCloud<PointXYZRGB >::Ptr Cloud);
		PointCloud<PointXYZRGB >::Ptr GetKeypoints(void) { return m_Keypoints; };

	private:
		bool ParseConfig(void);
		void SetParams(void);

		PointCloud<PointXYZRGB >::Ptr m_Keypoints;
		pcl::search::KdTree<PointXYZRGB >::Ptr m_FeatCloudKdTree;

		std::string m_ConfigFName;
		int m_KeypointType;
	
		SIFTKeypoint<PointXYZRGB, PointXYZRGB > m_SIFT;
		//! SIFT keypoint parameters
		std::bitset<4 > m_isSIFTAllSet;
		float m_SIFTMinScale;
		int m_SIFTOctaves;
		int m_SIFTScalesPerOctave;
		float m_SIFTMinContrast;

		UniformSampling<PointXYZRGB > m_Uniform;
		PointCloud<int > m_Uniform_Idx;
		//! Uniform sampling parameters
		std::bitset<1 > m_isUniformAllSet;
		float m_USamRadius;
};

#endif /* __FEATKEYPOINT_H__ */

