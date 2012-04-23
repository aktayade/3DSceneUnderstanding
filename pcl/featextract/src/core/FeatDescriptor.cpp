#include "FeatDescriptor.hpp"
#include "extern.hpp"

using namespace std;

FeatDescriptor::FeatDescriptor(void) : 
	m_KdTree(new pcl::search::KdTree<PointXYZRGB >),
	m_Normals(new PointCloud<Normal >)
{

}

FeatDescriptor::FeatDescriptor(const string& FeatureFile, const string& ConfigFName) :
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

bool FeatDescriptor::Compute(PointCloud<PointXYZRGB >::Ptr Cloud, vector<int > Indices)
{
	IndicesPtr indicesptr(new vector<int> (Indices));

	m_NormalEstimator.setInputCloud(Cloud);
	m_NormalEstimator.setSearchMethod(m_KdTree);

	m_NormalEstimator.setRadiusSearch(m_NormalEstRadius);
	m_NormalEstimator.compute(*m_Normals);
	// cout << "Number of indices in normal estimation: " << m_NormalEstimator.getIndices()->size() << endl;

	SpinImageEstimation<PointXYZRGB, Normal, Histogram<M_HISTGRAM_BINS > > m_SPIN(m_SPINImgWidth, m_SPINAngle, m_SPINMinPts);
	PointCloud<Histogram<M_HISTGRAM_BINS > >::Ptr spin_images(new PointCloud<Histogram<M_HISTGRAM_BINS > >);

	// Setup spin image computation
	m_SPIN.setInputCloud(Cloud);
	m_SPIN.setInputNormals(m_Normals);
	m_SPIN.setSearchMethod(m_KdTree);
	m_SPIN.setRadiusSearch(m_SPINRadius);
	m_SPIN.setIndices(indicesptr);

	// Compute SPIN
	m_SPIN.compute(*spin_images);

	FileStr.open(m_FeatureFile.c_str(), ios::app); // NOTE: Don't add the ios:trunc flag here!
	if(FileStr.is_open())
	{
		for (int i = 0; i < spin_images->points.size(); ++i)
		{
			pcl::Histogram<M_HISTGRAM_BINS > feat_line = spin_images->points[i];
			// Option 1: Avoid printing commas and brackets for Johnny's sake
			for(int i = 0; i < M_HISTGRAM_BINS; ++i)
			{
				if(i == 0)
					FileStr << "(";
				if(i == M_HISTGRAM_BINS - 1)
				{
					FileStr << feat_line.histogram[i] << ")" << endl;
					continue;
				}

				FileStr << feat_line.histogram[i] << " ";
			}

			// Option 2: Write to file using the overloaded << operator
			// FileStr << feat_line << endl;
		}
		FileStr.close();
	}
	else
		cout << "[WARNING]: Problem writing to file. Please check FeatDescriptor::Compute()" << endl;

	return true;
}

bool FeatDescriptor::Compute(PointCloud<PointXYZRGB >::Ptr Cloud, vector<int > Indices, vector<float > SegmentFeat, const int& SegmentLabel)
{
	if(SegmentLabel != int(SegmentFeat.at(2))) // Check for label correspondence
	{
		cout << "[FATAL]: Spin images not aligned with I+S features! Check logic." << endl;
		return false;
	}

	IndicesPtr indicesptr(new vector<int> (Indices));

	m_NormalEstimator.setInputCloud(Cloud);
	m_NormalEstimator.setSearchMethod(m_KdTree);

	m_NormalEstimator.setRadiusSearch(m_NormalEstRadius);
	m_NormalEstimator.compute(*m_Normals);
	// cout << "Number of indices in normal estimation: " << m_NormalEstimator.getIndices()->size() << endl;

	SpinImageEstimation<PointXYZRGB, Normal, Histogram<M_HISTGRAM_BINS > > m_SPIN(m_SPINImgWidth, m_SPINAngle, m_SPINMinPts);
	PointCloud<Histogram<M_HISTGRAM_BINS > >::Ptr spin_images(new PointCloud<Histogram<M_HISTGRAM_BINS > >);

	// Setup spin image computation
	m_SPIN.setInputCloud(Cloud);
	m_SPIN.setInputNormals(m_Normals);
	m_SPIN.setSearchMethod(m_KdTree);
	m_SPIN.setRadiusSearch(m_SPINRadius);
	m_SPIN.setIndices(indicesptr);

	// Compute SPIN
	m_SPIN.compute(*spin_images);

	FileStr.open(m_FeatureFile.c_str(), ios::app); // NOTE: Don't add the ios:trunc flag here!
	if(FileStr.is_open())
	{
		FileStr << "[";
		for(int k = 0; k < SegmentFeat.size(); ++k)
		{
			if(k != SegmentFeat.size() - 1)
			{
				FileStr << SegmentFeat.at(k) << " ";
				continue;
			}
					
			FileStr << SegmentFeat.at(k) << "]" << endl;
		}
		for(int i = 0; i < spin_images->points.size(); ++i)
		{
			pcl::Histogram<M_HISTGRAM_BINS > feat_line = spin_images->points[i];
			// Option 1: Avoid printing commas and brackets for Johnny's sake
			for(int i = 0; i < M_HISTGRAM_BINS; ++i)
			{
				if(i == 0)
					FileStr << "(";
				if(i == M_HISTGRAM_BINS - 1)
				{
					FileStr << feat_line.histogram[i] << ")" << endl;
					continue;
				}

				FileStr << feat_line.histogram[i] << " ";
			}

			// Option 2: Write to file using the overloaded << operator
			// FileStr << feat_line << endl;
		}
		FileStr.close();
	}
	else
		cout << "[WARNING]: Problem writing to file. Please check FeatDescriptor::Compute()" << endl;

	return true;
}

bool FeatDescriptor::ParseConfig(void)
{
	// Parse strategy: Tokenize the strings before and after the occurence of an '=' character in each line into keys and values
	// Then parse the value of each key
	ifstream ConfigFile;
	ConfigFile.open(m_ConfigFName.c_str());
	if(ConfigFile.is_open() == false)
	{
		cout << "ParseConfig() ERROR: Unable to open config file." << endl;
		return false;
	}

	vector<string > Keys;
	vector<string > Values;
	string Line;

	while(getline(ConfigFile, Line))
    {
        stringstream str(Line);
        string Key;
        string Value;

        if((getline(str, Key, '=')) && (str >> Value))
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
