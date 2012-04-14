#ifndef __FEATSEGMENT_H__
#define __FEATSEGMENT_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "extern.hpp"
#include "FeatPointCloud.hpp"

using namespace pcl;

class FeatSegment
{
	public:
		FeatSegment(const std::string& ConfigFName);
		virtual ~FeatSegment(void);
		void SetSceneCloud(PointCloud<PointXYZRGBCamSL >::Ptr SceneCloud) { m_SceneCloud = SceneCloud; };
		void BreakIntoSegments(void);
		std::vector<FeatPointCloud * > GetSegments(void) { return m_Segments; };
		std::vector<int > GetLabels(void) { return m_Labels; };

	private:
		std::vector<FeatPointCloud * > m_Segments;
		std::vector<int > m_Labels;
		PointCloud<PointXYZRGBCamSL >::Ptr m_SceneCloud;
		std::string m_ConfigFName;
};

#endif /* __FEATSEGMENT_H__ */



