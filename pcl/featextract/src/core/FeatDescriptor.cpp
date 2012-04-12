#include "FeatDescriptor.hpp"
#include <fstream>

using namespace std;

FeatDescriptor::FeatDescriptor(void) : 
	m_KdTree(new pcl::search::KdTree<PointXYZRGB >),
	m_Normals(new PointCloud<Normal >)
{

}

FeatDescriptor::FeatDescriptor(const int FeatType) :
	m_KdTree(new pcl::search::KdTree<PointXYZRGB >),
	m_Normals(new PointCloud<Normal >)
{

}

FeatDescriptor::~FeatDescriptor(void)
{

}

bool FeatDescriptor::Compute(PointCloud<PointXYZRGB >::Ptr Cloud, std::vector<int > Indices)
{
	IndicesPtr indicesptr(new std::vector<int> (Indices));

//	std::cout << "Setting some normal params." << std::endl;
	m_NormalEstimator.setInputCloud(Cloud);
	m_NormalEstimator.setSearchMethod(m_KdTree);
//	m_NormalEstimator.setIndices(indicesptr);

	m_NormalEstimator.setRadiusSearch(0.5); // PARAM
//	std::cout << "Computing normals." << std::endl;
	m_NormalEstimator.compute(*m_Normals);
//	std::cout << "Number of indices in normal estimation: " << m_NormalEstimator.getIndices()->size() << std::endl;

	SpinImageEstimation<PointXYZRGB, Normal, Histogram<153 > > m_SPIN(8, 0.5, 16); // PARAM
//	std::cout << "Setting some spin image params." << std::endl;
	// Setup spin image computation
	m_SPIN.setInputCloud(Cloud);
	m_SPIN.setInputNormals(m_Normals);

	// Use the same KdTree from the normal estimation
	m_SPIN.setSearchMethod(m_KdTree);
	// PARAM 153
	pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images(new pcl::PointCloud<pcl::Histogram<153> >);
	m_SPIN.setRadiusSearch(2.0); // PARAM

	// Actually compute the spin images
	m_SPIN.setIndices(indicesptr);
//	std::cout << "Number of indices in SPIN image estimation: " << m_SPIN.getIndices()->size() << std::endl;
//	std::cout << "Computing spin image." << std::endl;
	m_SPIN.compute(*spin_images);
//	std::cout << "SI output points.size (): " << spin_images->points.size () << std::endl;

//	fstream FileStr;
//	FileStr.open("features.dat", fstream::in | fstream::out | fstream::app);

//	for (int i = 0; i < spin_images->points.size(); ++i)
//	{
//		pcl::Histogram<153> feat_line = spin_images->points[i];
//		FileStr << feat_line << std::endl;
//	}

//	FileStr.close();
}

