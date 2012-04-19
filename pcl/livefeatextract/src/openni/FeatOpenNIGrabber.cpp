#include "FeatOpenNIGrabber.hpp"

FeatOpenNIGrabber::FeatOpenNIGrabber() : 
	m_Viewer("FeatExtract PC Grabber")
{

}

void FeatOpenNIGrabber::ShowCloud(const pcl::PointCloud<pcl::PointXYZRGBA >::ConstPtr& Cloud)
{
	if(!m_Viewer.wasStopped())
		m_Viewer.showCloud(Cloud);
}

void FeatOpenNIGrabber::Run(void)
{
	pcl::Grabber * Interface = new pcl::OpenNIGrabber();

	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> Func = boost::bind(&FeatOpenNIGrabber::ShowCloud, this, _1);

	Interface->registerCallback(Func);
	Interface->start();

	while(!m_Viewer.wasStopped())
	{
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}

	Interface->stop();
}
