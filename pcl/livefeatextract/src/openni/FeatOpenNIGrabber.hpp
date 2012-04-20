#ifndef __FEATOPENNIGRABBER_H__
#define __FEATOPENNIGRABBER_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#ifdef _WIN32
	#define sleep(x) Sleep((x)*1000)
#endif

using namespace pcl;

class FeatOpenNIGrabber
{
	public:
		FeatOpenNIGrabber();
		void Run(void);
		PointCloud<PointXYZRGB >::Ptr GetCurrentCloud(void) { return m_CurrentCloud; };

	private:
		pcl::visualization::CloudViewer m_Viewer;
		PointCloud<PointXYZRGB >::Ptr m_CurrentCloud;

		void ShowCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& Cloud);
};

#endif /* __FEATOPENNIGRABBER_H__ */
