#include "FeatKeypoint.hpp"

FeatKeypoint::FeatKeypoint(const std::string& ConfigFName) :
	m_Keypoints(new PointCloud<PointXYZRGB >),
	m_FeatCloudKdTree(new pcl::search::KdTree<PointXYZRGB >)
{
	m_ConfigFName = ConfigFName;
	ParseConfig();
	SetParams();
}

FeatKeypoint::~FeatKeypoint(void)
{

}

void FeatKeypoint::SetParams(void)
{
	if(m_KeypointType == M_KEYPOINT_SIFT)
	{
		if(m_isSIFTAllSet.count() != m_isSIFTAllSet.size())
		{
			std::cout << "Some SIFT parameters not set. Using default values." << std::endl;
			m_SIFTMinScale = 0.01;
			m_SIFTOctaves = 3;
			m_SIFTScalesPerOctave = 3;
			m_SIFTMinContrast = 10.0;
		}

		// Print for debug purposes
		//	std::cout << std::endl <<
		//		m_SIFTMinScale << std::endl << 
		//		m_SIFTOctaves << std::endl << 
		//		m_SIFTScalesPerOctave << std::endl << 
		//		m_SIFTMinContrast << std::endl;

		// Set the stored parameters
		m_SIFT.setScales(m_SIFTMinScale, m_SIFTOctaves, m_SIFTScalesPerOctave);
		m_SIFT.setMinimumContrast(m_SIFTMinContrast);
		m_SIFT.setSearchMethod(m_FeatCloudKdTree);
	}
	else if(m_KeypointType == M_KEYPOINT_UNIFORM)
	{
		if(m_isUniformAllSet.count() != m_isUniformAllSet.size())
		{
			std::cout << "Some UniformSampling parameters not set. Using default values." << std::endl;
			m_USamRadius = 0.01;
		}

		// Set the stored parameters
		m_Uniform.setRadiusSearch(m_USamRadius);
		m_Uniform.setSearchMethod(m_FeatCloudKdTree);
	}
}

bool FeatKeypoint::Compute(PointCloud<PointXYZRGB >::Ptr Cloud)
{
	if(m_KeypointType == M_KEYPOINT_SIFT)
	{
		m_SIFT.setInputCloud(Cloud);
		m_SIFT.compute(*m_Keypoints);
	//	std::cout << "Cloud size\t" << Cloud->points.size() << "\tSIFT Keypoints\t" << m_Keypoints->points.size() << std::std::endl;
	}
	else if(m_KeypointType == M_KEYPOINT_UNIFORM)
	{
		m_Uniform.setInputCloud(Cloud);
		m_Uniform.compute(m_Uniform_Idx);
		copyPointCloud<PointXYZRGB, PointXYZRGB >(*Cloud, m_Uniform_Idx.points, *m_Keypoints); 
	}

	return true;
}

bool FeatKeypoint::ParseConfig(void)
{
	// Parse strategy: Tokenize the strings before and after the occurence of an '=' character in each line into keys and values
	// Then parse the value of each key
	std::ifstream ConfigFile;
	ConfigFile.open(m_ConfigFName.c_str());
	if(ConfigFile.is_open() == false)
	{
		std::cerr << "ParseConfig() ERROR: Unable to open config file." << std::endl;
		return false;
	}

	std::vector<std::string > Keys;
	std::vector<std::string > Values;
	std::string Line;

	while(std::getline(ConfigFile, Line))
    {
        std::stringstream str(Line);
        std::string Key;
        std::string Value;

        if((std::getline(str, Key, '=')) && (str >> Value))
        {
			// Look for KeypointType first. Only then start looking at the parameters
			if(strcmp(Key.c_str(), "KeypointType") == 0)
			{
				m_KeypointType = atoi(Value.c_str());
			}

			if(m_KeypointType == M_KEYPOINT_SIFT)
			{
				if(strcmp(Key.c_str(), "SIFTMinScale") == 0)
				{
					m_isSIFTAllSet.set(0);
					m_SIFTMinScale = atof(Value.c_str());
				}
				else if(strcmp(Key.c_str(), "SIFTOctaves") == 0)
				{
					m_isSIFTAllSet.set(1);
					m_SIFTOctaves = atoi(Value.c_str());
				}
				else if(strcmp(Key.c_str(), "SIFTScalesPerOctave") == 0)
				{
					m_isSIFTAllSet.set(2);
					m_SIFTScalesPerOctave = atoi(Value.c_str());
				}
				else if(strcmp(Key.c_str(), "SIFTMinContrast") == 0)
				{
					m_isSIFTAllSet.set(3);
					m_SIFTMinContrast = atof(Value.c_str());
				}
			}
			else if(m_KeypointType == M_KEYPOINT_UNIFORM)
			{
				if(strcmp(Key.c_str(), "USamRadius") == 0)
				{
					m_isUniformAllSet.set(0);
					m_USamRadius = atof(Value.c_str());
				}
			}
        }
    }

	ConfigFile.close();
	return true;
}
