#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

int
main (int, char** argv)
{
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile <pcl::PointXYZ> (filename.c_str (), *cloud) == -1)
  // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }
  std::cout << "Loaded " << cloud->points.size () << " points." << std::endl;

  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimation.setSearchMethod (kdtree);

  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
  normal_estimation.setRadiusSearch (0.03);
  normal_estimation.compute (*normals);

  // Setup spin image computation
  pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > spin_image_descriptor(8, 0.5, 16);
  spin_image_descriptor.setInputCloud (cloud);
  spin_image_descriptor.setInputNormals (normals);

  // Use the same KdTree from the normal estimation
  spin_image_descriptor.setSearchMethod (kdtree);
  pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
  spin_image_descriptor.setRadiusSearch (0.2);

  // Actually compute the spin images
  spin_image_descriptor.compute (*spin_images);
  std::cout << "SI output points.size (): " << spin_images->points.size () << std::endl;

  // Display and retrieve the spin image descriptor vector for the first point.
  pcl::Histogram<153> first_descriptor = spin_images->points[0];
  std::cout << first_descriptor << std::endl;

  return 0;
}
