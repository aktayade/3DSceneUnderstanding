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

	std::cout << "Starting test\n";
	PointCloud<PointXYZRGB >::Ptr incloud(new PointCloud<PointXYZRGB >);
	PointCloud<PointXYZRGB >::Ptr outcloud(new PointCloud<PointXYZRGB >);
	pcl::copyPointCloud(*m_SceneCloud, *incloud);
	segment(incloud, outcloud);
	std::cout << "\nNum segments is: " << m_Segments.size() << "\nEnding test\n";
}

void extractEuclideanClusters (
      PointCloud<PointXYZRGB >::Ptr cloud, pcl::PointCloud<pcl::Normal >::Ptr normals,
      pcl::search::KdTree<PointXYZRGB >::Ptr tree, 
      float tolerance, std::vector<pcl::PointIndices > &clusters, double eps_angle,
      unsigned int min_pts_per_cluster = 1,
      unsigned int max_pts_per_cluster = (std::numeric_limits<int >::max) ())
  {
    // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
    //and indices[i]
    float adjTolerance = 0;

    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool > processed(cloud->points.size(), false);

	std::vector<int> nn_indices;
	std::vector<float> nn_distances;
	// Process all points in the indices vector
	std::cout << "Point size is " << cloud->points.size () << std::endl;
	for (size_t i = 0; i < cloud->points.size (); ++i)
    {
		if(processed[i])
			continue;

		std::vector<int > seed_queue;
		int sq_idx = 0;
		seed_queue.push_back(i);
		processed[i] = true;

		int cnt = 0;

		while (sq_idx < (int)seed_queue.size())
		{ 
			cnt++;

			// Search for sq_idx
//			 adjTolerance = cloud->points[seed_queue[sq_idx]].distance * tolerance;
			adjTolerance = tolerance;

			if (!tree->radiusSearch(seed_queue[sq_idx], adjTolerance, nn_indices, nn_distances))
	        {
				sq_idx++;
				continue;
			}

			for(size_t j = 1; j < nn_indices.size (); ++j) // nn_indices[0] should be sq_idx
			{
				if (processed[nn_indices[j]]) // Has this point been processed before ?
					continue;

				processed[nn_indices[j]] = true;
				// [-1;1]
				double dot_p =
				normals->points[i].normal[0] * normals->points[nn_indices[j]].normal[0] +
				normals->points[i].normal[1] * normals->points[nn_indices[j]].normal[1] +
				normals->points[i].normal[2] * normals->points[nn_indices[j]].normal[2];
				if ( fabs (acos (dot_p)) < eps_angle )
				{
					processed[nn_indices[j]] = true;
					seed_queue.push_back (nn_indices[j]);
				}
	        }

			sq_idx++;
		}

		// If this queue is satisfactory, add to the clusters
		if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
		{
			pcl::PointIndices r;
			r.indices.resize (seed_queue.size ());
			for (size_t j = 0; j < seed_queue.size (); ++j)
				r.indices[j] = seed_queue[j];

			sort (r.indices.begin (), r.indices.end ());
			r.indices.erase (unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

			r.header = cloud->header;
			//ROS_INFO ("cluster of size %d data point\n ",r.indices.size());
			clusters.push_back(r);
		}
	}
}

void FeatSegment::segment(const PointCloud<PointXYZRGB >::Ptr cloud,  PointCloud<PointXYZRGB >::Ptr outcloud)
{
	// PARAM - Lots of them
	int min_pts_per_cluster = 1;
	int max_pts_per_cluster = INT_MAX; //3000000;
	assert(max_pts_per_cluster > 3000000); // Avoid overflow
    int number_neighbours = 50; // PARAM
    float radius = 0.025; // 0.025 PARAM
    float angle = 0.52; // PARAM

	// Required KdTrees
    pcl::search::KdTree<PointXYZRGB >::Ptr NormalsTree(new pcl::search::KdTree<PointXYZRGB >);
    pcl::search::KdTree<PointXYZRGB >::Ptr ClustersTree(new pcl::search::KdTree<PointXYZRGB >);
    PointCloud<Normal >::Ptr CloudNormals(new PointCloud<Normal >);
    NormalEstimation<PointXYZRGB, Normal > NormalsFinder;
    std::vector<PointIndices > clusters;

	ClustersTree->setInputCloud(cloud);

	// Set normal estimation parameters
    NormalsFinder.setKSearch(number_neighbours);
    NormalsFinder.setSearchMethod(NormalsTree);
    NormalsFinder.setInputCloud(cloud);
    NormalsFinder.compute(*CloudNormals);

    extractEuclideanClusters(cloud, CloudNormals, ClustersTree, radius, clusters, angle, min_pts_per_cluster, max_pts_per_cluster);
    fprintf(stderr, "Number of clusters found matching the given constraints: %d.", (int)clusters.size ());

	FileStr.open("live_segments.txt", ios::app); // NOTE: Don't add the ios:trunc flag here!
	// Copy to clusters to segments
	for (size_t i = 0; i < clusters.size (); ++i)
	{
		if(FileStr.is_open())
		{
			FileStr << i << endl;
			for(int j = 0; j < clusters[i].indices.size(); ++j)
				FileStr << clusters[i].indices.at(j) << "\t";
			FileStr << endl;

			FeatPointCloud * Segment = new FeatPointCloud(m_SceneCloud, clusters[i].indices, m_ConfigFName);
			m_Segments.push_back(Segment);
		}
	}
	FileStr.close();
}

