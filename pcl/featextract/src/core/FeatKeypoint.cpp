#include "FeatKeypoint.hpp"

FeatKeypoint::FeatKeypoint(const std::string& ConfigFName) :
	m_Keypoints(new PointCloud<PointWithScale >),
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
	if(m_isAllSet.count() != m_isAllSet.size())
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

	// Set some default initial parameters
	m_SIFT.setScales(m_SIFTMinScale, m_SIFTOctaves, m_SIFTScalesPerOctave);
	m_SIFT.setMinimumContrast(m_SIFTMinContrast);
	m_SIFT.setSearchMethod(m_FeatCloudKdTree);
}

bool FeatKeypoint::Compute(PointCloud<PointXYZRGB >::Ptr Cloud)
{
	m_SIFT.setInputCloud(Cloud);
	m_SIFT.compute(*m_Keypoints);
//	std::cout << "Cloud size\t" << Cloud->points.size() << "\tSIFT Keypoints\t" << m_Keypoints->points.size() << std::std::endl;

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
			if(strcmp(Key.c_str(), "SIFTMinScale") == 0)
			{
				m_isAllSet.set(0);
				m_SIFTMinScale = atof(Value.c_str());
			}
			else if(strcmp(Key.c_str(), "SIFTOctaves") == 0)
			{
				m_isAllSet.set(1);
				m_SIFTOctaves = atoi(Value.c_str());
			}
			else if(strcmp(Key.c_str(), "SIFTScalesPerOctave") == 0)
			{
				m_isAllSet.set(2);
				m_SIFTScalesPerOctave = atoi(Value.c_str());
			}
			else if(strcmp(Key.c_str(), "SIFTMinContrast") == 0)
			{
				m_isAllSet.set(3);
				m_SIFTMinContrast = atof(Value.c_str());
			}
        }
    }

	ConfigFile.close();
	return true;
}
