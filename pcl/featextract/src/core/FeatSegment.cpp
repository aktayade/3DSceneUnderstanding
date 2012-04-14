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
	
	for (int INDseg = 1; INDseg <= NUMseg; ++INDseg)
	{
//		std::cout << "Segment number: " << std::setw(4) << INDseg;
      	std::vector<int > PtIndSeg;

		int tmp;
      	for(int i = 0; i < m_SceneCloud->points.size (); ++i)
      	{
      		if(m_SceneCloud->points[i].segment == INDseg)
			{
      			PtIndSeg.push_back(i);
				tmp = m_SceneCloud->points[i].label;
			}
      	}
		m_Labels.push_back(tmp);

		FeatPointCloud * Segment = new FeatPointCloud(m_SceneCloud, PtIndSeg, m_ConfigFName);
//		std::cout << "   Size: " << std::setw(5) << Segment->GetCloud()->points.size() << std::endl;
		m_Segments.push_back(Segment);
	}
}

