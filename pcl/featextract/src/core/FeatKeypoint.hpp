#ifndef __FEATKEYPOINT_H__
#define __FEATKEYPOINT_H__

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace pcl;

class FeatKeypoint
{
	public:
		FeatKeypoint(void);
		virtual ~FeatKeypoint();

		bool Compute(PointCloud<PointXYZRGB >::Ptr Cloud);
		PointCloud<PointWithScale >::Ptr GetKeypoints(void) { return m_Keypoints; };

	private:
		PointCloud<PointWithScale >::Ptr m_Keypoints;
		pcl::search::KdTree<PointXYZRGB >::Ptr m_FeatCloudKdTree;
	
		SIFTKeypoint<PointXYZRGB, PointWithScale > m_SIFT;
};

#endif /* __FEATKEYPOINT_H__ */

