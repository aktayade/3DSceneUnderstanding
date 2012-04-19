#include "FeatSegment.hpp"

FeatSegment::FeatSegment(const std::string& ConfigFName)
{
	m_ConfigFName = ConfigFName;
}

FeatSegment::~FeatSegment(void)
{

}

void FeatSegment::BreakIntoSegments(void)
{
	if(m_SceneCloud == NULL)
	{
		std::cerr << "Error. No scene point cloud set. Use SetSceneCloud() to do that." << std::endl;
		return;
	}

	int NUMseg;
	int NUMpts = m_SceneCloud->points.size (); // TODO change m_SceneCloud to PointXYZRGBCamSl
	NUMseg = m_SceneCloud->points[NUMpts - 1].segment;
	std::cout << "Number of segments: " << NUMseg << std::endl;
	
	for (int i = 1; i <= NUMseg; ++i)
	{
//		std::cout << "Segment number: " << std::setw(4) << i;
      	std::vector<int > PtIndSeg;

		int tmp;
      	for(int j = 0; j < m_SceneCloud->points.size (); ++j)
      	{
      		if(m_SceneCloud->points[j].segment == i)
			{
      			PtIndSeg.push_back(j);
				tmp = m_SceneCloud->points[j].label;
			}
      	}
		m_Labels.push_back(tmp);

//		if(PtIndSeg.size() <= 1)
//			std::cout << "Found segment " << i << " with " << PtIndSeg.size() << " points." << std::endl;

		FeatPointCloud * Segment = new FeatPointCloud(m_SceneCloud, PtIndSeg, m_ConfigFName);
//		std::cout << "   Size: " << std::setw(5) << Segment->GetCloud()->points.size() << std::endl;
		m_Segments.push_back(Segment);
	}
}

