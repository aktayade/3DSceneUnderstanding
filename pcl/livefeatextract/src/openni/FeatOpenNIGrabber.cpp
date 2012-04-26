#include "FeatOpenNIGrabber.hpp"

FeatOpenNIGrabber::FeatOpenNIGrabber() : 
	m_Viewer("FeatExtract PC Grabber"),
	m_CurrentCloud(new PointCloud<PointXYZRGB >)
{

}

void FeatOpenNIGrabber::ShowCloud(const pcl::PointCloud<pcl::PointXYZRGBA >::ConstPtr& Cloud)
{
	copyPointCloud<PointXYZRGBA, PointXYZRGB >(*Cloud, *m_CurrentCloud);
	if(isTakeSnapshot == true)
	{
		pcl::PCDWriter Writer;
		if(Writer.writeBinary<PointXYZRGB >("live.pcd", *m_CurrentCloud) == -1)
		{
			cout << "[ERROR]: Problem writing PCD file." << endl;
			return;
		}
		cout << "Snapshot written to file." << endl;
		isTerminate = true;
		isTakeSnapshot = false;
		return;
	}

	if(!m_Viewer.wasStopped())
		m_Viewer.showCloud(Cloud);
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	if(event.getKeySym() == "m" && event.keyDown())
		isTakeSnapshot = true;
}

void FeatOpenNIGrabber::Run(void)
{
	pcl::Grabber * Interface = new pcl::OpenNIGrabber();

	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> Func = boost::bind(&FeatOpenNIGrabber::ShowCloud, this, _1);

	Interface->registerCallback(Func);
	Interface->start();

	m_Viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&m_Viewer);

	while(!m_Viewer.wasStopped() && isTerminate == false)
	{
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}

	Interface->stop();
}
