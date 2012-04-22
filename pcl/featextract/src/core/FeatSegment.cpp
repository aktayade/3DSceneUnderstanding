#include "FeatSegment.hpp"

FeatSegment::FeatSegment(const string& ConfigFName)
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
		cerr << "[FATAL]: Error. No scene point cloud set. Use SetSceneCloud() to do that." << endl;
		return;
	}

	int NUMseg;
	int NUMpts = m_SceneCloud->points.size();
	NUMseg = m_SceneCloud->points[NUMpts - 1].segment;
	cout << "[INFO]: Number of segments: " << NUMseg << endl;
	
	for (int i = 1; i <= NUMseg; ++i)
	{
//		cout << "Segment number: " << setw(4) << i;
      	vector<int > PtIndSeg;

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
//			cout << "Found segment " << i << " with " << PtIndSeg.size() << " points." << endl;

		FeatPointCloud * Segment = new FeatPointCloud(m_SceneCloud, PtIndSeg, m_ConfigFName);
//		cout << "   Size: " << setw(5) << Segment->GetCloud()->points.size() << endl;
		m_Segments.push_back(Segment);
	}
}

void FeatSegment::BreakIntoSegments(const string& NodeFeatsFile, const int& SceneNumber)
{
	if(m_SceneCloud == NULL)
	{
		cerr << "[FATAL]: Error. No scene point cloud set. Use SetSceneCloud() to do that." << endl;
		return;
	}

	ifstream NodeFeatsFileStr;
	NodeFeatsFileStr.open(NodeFeatsFile.c_str(), ios::in);
	if(NodeFeatsFileStr.is_open() == false)
	{
		cerr << "[WARNING]: Unable to open node features file. Reverting to just Spin images with no Image + Shape Features." << endl;
		BreakIntoSegments();
		return;
	}

	int NUMseg;
	int NUMpts = m_SceneCloud->points.size();
	NUMseg = m_SceneCloud->points[NUMpts - 1].segment;
	cout << "[INFO]: Number of segments: " << NUMseg << endl;
	
	for (int i = 1; i <= NUMseg; ++i)
	{
		// Parse the nodefeatsfile
		string Line;
	    vector<float > ISData(55); //! See the Cornell features file for details about 55
		while(getline(NodeFeatsFileStr, Line))
		{
		    if(Line[0] == '#' || Line[0] == '@')
				continue;

			stringstream LineStr(stringstream::in | stringstream::out);
			LineStr << Line;
			for(int n = 0; n < ISData.size(); ++n)
				LineStr >> ISData.at(n);

			if(int(ISData.at(0)) != SceneNumber) // Check for scene number
			{
				fill(ISData.begin(), ISData.end(), 0.0f);
				continue;
			}

			if(int(ISData.at(1)) == i) // Check for segment number	
				break;

			fill(ISData.begin(), ISData.end(), 0.0f);
		}
		m_SegISFeatures.push_back(ISData); // Skip first 3 numbers
//		cout << "Segment: " << i << endl;
//		for (int k = 0; k < ISData.size(); k += 1)
//		{
//			cout << ISData.at(k) << "\t";
//		}
//		cout << endl;
//		exit(-1);

//		cout << "Segment number: " << setw(4) << i;
      	vector<int > PtIndSeg;

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
//			cout << "Found segment " << i << " with " << PtIndSeg.size() << " points." << endl;

		FeatPointCloud * Segment = new FeatPointCloud(m_SceneCloud, PtIndSeg, m_ConfigFName);
//		cout << "   Size: " << setw(5) << Segment->GetCloud()->points.size() << endl;
		m_Segments.push_back(Segment);

		// Go back to beginning of file
		NodeFeatsFileStr.clear();
		NodeFeatsFileStr.seekg(0, ios::beg);
	}

	NodeFeatsFileStr.close();
}


