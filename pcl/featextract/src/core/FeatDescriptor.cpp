#include "FeatDescriptor.hpp"

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
	std::cout << "Setting some normal params." << std::endl;
	m_NormalEstimator.setInputCloud(Cloud);
	m_NormalEstimator.setSearchMethod(m_KdTree);

	m_NormalEstimator.setRadiusSearch(2.0);
	std::cout << "Computing normals." << std::endl;
	m_NormalEstimator.compute(*m_Normals);

	SpinImageEstimation<PointXYZRGB, Normal, Histogram<153 > > m_SPIN(8, 0.5, 16);
	std::cout << "Setting some spin image params." << std::endl;
	// Setup spin image computation
	m_SPIN.setInputCloud(Cloud);
	m_SPIN.setInputNormals(m_Normals);

	// Use the same KdTree from the normal estimation
	m_SPIN.setSearchMethod(m_KdTree);
	pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images(new pcl::PointCloud<pcl::Histogram<153> >);
	m_SPIN.setRadiusSearch(2.0);

	// Actually compute the spin images
	IndicesPtr indicesptr(new std::vector<int> (Indices));
	m_SPIN.setIndices(indicesptr);
	std::cout << "Computing spin image." << std::endl;
	m_SPIN.compute(*spin_images);
	std::cout << "SI output points.size (): " << spin_images->points.size () << std::endl;

	// Display and retrieve the spin image descriptor vector for the first point.
	pcl::Histogram<153> first_descriptor = spin_images->points[0];
	std::cout << first_descriptor << std::endl;
}

