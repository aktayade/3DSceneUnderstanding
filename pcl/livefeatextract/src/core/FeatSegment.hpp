#ifndef __FEATSEGMENT_H__
#define __FEATSEGMENT_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

#include "extern.hpp"
#include "FeatPointCloud.hpp"

using namespace pcl;
using namespace std;

class FeatSegment
{
	public:
		FeatSegment(const std::string& ConfigFName);
		virtual ~FeatSegment(void);
		void SetSceneCloud(PointCloud<PointXYZRGB >::Ptr SceneCloud) { m_SceneCloud = SceneCloud; };
		void BreakIntoSegments(void);
		std::vector<FeatPointCloud * > GetSegments(void) { return m_Segments; };

	private:
		std::vector<FeatPointCloud * > m_Segments;
		PointCloud<PointXYZRGB >::Ptr m_SceneCloud;
		std::string m_ConfigFName;

		void segment(const PointCloud<PointXYZRGB >::Ptr cloud,  PointCloud<PointXYZRGB >::Ptr outcloud);
};

#endif /* __FEATSEGMENT_H__ */



