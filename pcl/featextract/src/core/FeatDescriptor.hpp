#ifndef __FEATDESCRIPTOR_H__
#define __FEATDESCRIPTOR_H__

#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

#define FEATEXTRACT_SPIN 0

using namespace pcl;

class FeatDescriptor
{
	public:
		FeatDescriptor(void);
		FeatDescriptor(const int FeatType);
		virtual ~FeatDescriptor(void);

		bool Compute(PointCloud<PointXYZRGB >::Ptr Cloud, std::vector<int > Indices);

	private:
		NormalEstimation<PointXYZRGB, Normal > m_NormalEstimator;
		pcl::search::KdTree<PointXYZRGB >::Ptr m_KdTree;
		PointCloud<Normal >::Ptr m_Normals;
};

#endif /* __FEATDESCRIPTOR_H__ */

