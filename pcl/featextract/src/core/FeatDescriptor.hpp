#ifndef __FEATDESCRIPTOR_H__
#define __FEATDESCRIPTOR_H__

#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

#define M_HISTGRAM_BINS 153 // PARAM

using namespace pcl;

class FeatDescriptor
{
	public:
		FeatDescriptor(void);
		FeatDescriptor(const std::string& FeatureFile, const std::string& ConfigFName);
		FeatDescriptor(const int FeatType);
		virtual ~FeatDescriptor(void);

		bool Compute(PointCloud<PointXYZRGB >::Ptr Cloud, std::vector<int > Indices);
		bool ParseConfig(void);

	private:
		NormalEstimation<PointXYZRGB, Normal > m_NormalEstimator;
		pcl::search::KdTree<PointXYZRGB >::Ptr m_KdTree;
		PointCloud<Normal >::Ptr m_Normals;

		std::string m_FeatureFile;

		std::string m_ConfigFName;
		//! SPIN Image and normal parameters
		std::bitset<5 > m_isAllParamSet;
		float m_NormalEstRadius;
		int m_SPINImgWidth;
		float m_SPINAngle;
		int m_SPINMinPts;
		float m_SPINRadius;
};

#endif /* __FEATDESCRIPTOR_H__ */

