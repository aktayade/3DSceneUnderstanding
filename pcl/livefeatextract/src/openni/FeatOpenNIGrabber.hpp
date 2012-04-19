#ifndef __FEATOPENNIGRABBER_H__
#define __FEATOPENNIGRABBER_H__

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#ifdef _WIN32
	#define sleep(x) Sleep((x)*1000)
#endif

class FeatOpenNIGrabber
{
	public:
		FeatOpenNIGrabber(); : m_Viewer("FeatExtract PC Grabber") {}
		void Run(void);

	private:
		pcl::visualization::CloudViewer m_Viewer;

		void ShowCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& Cloud);
};

#endif /* __FEATOPENNIGRABBER_H__ */
