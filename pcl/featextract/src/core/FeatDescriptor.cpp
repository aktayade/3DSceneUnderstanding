#include "FeatDescriptor.hpp"
#include "extern.hpp"

using namespace std;

FeatDescriptor::FeatDescriptor(void) : 
	m_KdTree(new pcl::search::KdTree<PointXYZRGB >),
	m_Normals(new PointCloud<Normal >)
{

}

FeatDescriptor::FeatDescriptor(const std::string& FeatureFile, const std::string& ConfigFName) :
	m_KdTree(new pcl::search::KdTree<PointXYZRGB >),
	m_Normals(new PointCloud<Normal >)
{
	m_ConfigFName = ConfigFName;
	ParseConfig();
	if(m_isAllParamSet.count() != m_isAllParamSet.size())
	{
		cout << "Some feature parameters not set. Using default values." << endl;
		m_NormalEstRadius = 0.01;
		m_SPINImgWidth = 8;
		m_SPINAngle = 0.1;
		m_SPINMinPts = 4;
		m_SPINRadius = 0.1;
	}

	// Print for testing purposes
//	cout << m_NormalEstRadius << endl <<
//		m_SPINImgWidth << endl <<
//		m_SPINAngle << endl <<
//		m_SPINMinPts << endl <<
//		m_SPINRadius << endl;


	m_FeatureFile = FeatureFile;
}

FeatDescriptor::~FeatDescriptor(void)
{

}

bool FeatDescriptor::Compute(PointCloud<PointXYZRGB >::Ptr Cloud, std::vector<int > Indices)
{
	IndicesPtr indicesptr(new std::vector<int> (Indices));

//	std::cout << "Setting some normal params." << std::std::endl;
	m_NormalEstimator.setInputCloud(Cloud);
	m_NormalEstimator.setSearchMethod(m_KdTree);
//	m_NormalEstimator.setIndices(indicesptr);

	m_NormalEstimator.setRadiusSearch(m_NormalEstRadius); // PARAM - Using 1cm for now (just like the Cornell guys)
//	std::cout << "Computing normals." << std::std::endl;
	m_NormalEstimator.compute(*m_Normals);
//	std::cout << "Number of indices in normal estimation: " << m_NormalEstimator.getIndices()->size() << std::std::endl;

	SpinImageEstimation<PointXYZRGB, Normal, Histogram<153 > > m_SPIN(m_SPINImgWidth, m_SPINAngle, m_SPINMinPts); // PARAM
//	std::cout << "Setting some spin image params." << std::std::endl;
	// Setup spin image computation
	m_SPIN.setInputCloud(Cloud);
	m_SPIN.setInputNormals(m_Normals);

	// Use the same KdTree from the normal estimation
	m_SPIN.setSearchMethod(m_KdTree);
	// PARAM 153
	pcl::PointCloud<pcl::Histogram<153 > >::Ptr spin_images(new pcl::PointCloud<pcl::Histogram<153 > >);
	m_SPIN.setRadiusSearch(m_SPINRadius); // PARAM

	// Actually compute the spin images
	m_SPIN.setIndices(indicesptr);
//	std::cout << "Number of indices in SPIN image estimation: " << m_SPIN.getIndices()->size() << std::std::endl;
//	std::cout << "Computing spin image." << std::std::endl;
	m_SPIN.compute(*spin_images);
//	std::cout << "SI output points.size (): " << spin_images->points.size () << std::std::endl;

	// NOTE: Don't add the ios:trunc flag here!
	FileStr.open(m_FeatureFile.c_str(), ios::app);
	if(FileStr.is_open())
	{
		for (int i = 0; i < spin_images->points.size(); ++i)
		{
			pcl::Histogram<153 > feat_line = spin_images->points[i];
			// Option 1: Avoid printing commas and brackets for Johnny's sake
			for(int i = 0; i < 153; ++i) // TODO Why 153 and not 152 here? But checked with the << for Histogram
			{
				FileStr << feat_line.histogram[i] << " ";
			}
			FileStr << std::endl;

			// Option 2: Write to file using the overloaded << operator
			// FileStr << feat_line << std::endl;
		}
		FileStr.close();
	}
	else
		std::cout << "Problem writing to file. Please check FeatDescriptor::Compute()" << std::endl;

	return true;
}

bool FeatDescriptor::ParseConfig(void)
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
			if(strcmp(Key.c_str(), "NormalEstRadius") == 0)
			{
				m_isAllParamSet.set(0);
				m_NormalEstRadius = atof(Value.c_str());
			}
			else if(strcmp(Key.c_str(), "SPINImgWidth") == 0)
			{
				m_isAllParamSet.set(1);
				m_SPINImgWidth = atoi(Value.c_str());
			}
			else if(strcmp(Key.c_str(), "SPINAngle") == 0)
			{
				m_isAllParamSet.set(2);
				m_SPINAngle = atof(Value.c_str());
			}
			else if(strcmp(Key.c_str(), "SPINMinPts") == 0)
			{
				m_isAllParamSet.set(3);
				m_SPINMinPts = atoi(Value.c_str());
			}
			else if(strcmp(Key.c_str(), "SPINRadius") == 0)
			{
				m_isAllParamSet.set(4);
				m_SPINRadius = atof(Value.c_str());
			}
        }
    }

	ConfigFile.close();
	return true;
}
