#ifndef __FEATSEGMENT_H__
#define __FEATSEGMENT_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <sstream>
#include <algorithm>

#include "extern.hpp"
#include "FeatPointCloud.hpp"

using namespace pcl;
using namespace std;

class FeatSegment
{
	public:
		FeatSegment(const std::string& ConfigFName);
		virtual ~FeatSegment(void);
		void SetSceneCloud(PointCloud<PointXYZRGBCamSL >::Ptr SceneCloud) { m_SceneCloud = SceneCloud; };
		void BreakIntoSegments(void);
		void BreakIntoSegments(const std::string& NodeFeatsFile, const int& SceneNumber);
		std::vector<FeatPointCloud * > GetSegments(void) { return m_Segments; };
		std::vector<int > GetLabels(void) { return m_Labels; };
		std::vector<std::vector<float > > GetISFeatures(void) { return m_SegISFeatures; };

	private:
		std::vector<FeatPointCloud * > m_Segments;
		std::vector<int > m_Labels; //!< Segment labels stored in the same order as m_Segments
		std::vector<std::vector<float > > m_SegISFeatures; //!< Image + Shape features from the Cornell paper
		int m_SegISFeaturesLen; //!< The length of the each I+S feature
		PointCloud<PointXYZRGBCamSL >::Ptr m_SceneCloud;
		std::string m_ConfigFName;
};

#endif /* __FEATSEGMENT_H__ */



