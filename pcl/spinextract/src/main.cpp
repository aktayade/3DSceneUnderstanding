#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

// Constants from http://pastebin.com/8s5k8M1A
const float min_scale = 0.01;
const int nr_octaves = 3;
const int nr_scales_per_octave = 3;
const float min_contrast = 10.0;

int main (int, char ** argv)
{
  // Load the point cloud data
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointWithScale>::Ptr siftcloud(new PointCloud<PointWithScale>);

  if (pcl::io::loadPCDFile<PointXYZRGB>(argv[1], *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file");
    return -1;
  }
  cout << "Loaded " << cloud->points.size () << " points." << endl;

  // STEP 1: Detect SIFT 3D Keypoints in the point cloud
  SIFTKeypoint<PointXYZRGB, PointWithScale > SIFTPoints;

  // Use a non-FLANN-based KdTree to perform neighborhood searches
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  SIFTPoints.setSearchMethod(kdtree);

  // Set the detection parameters
  SIFTPoints.setScales(min_scale, nr_octaves, nr_scales_per_octave);
  SIFTPoints.setMinimumContrast(min_contrast);

  SIFTPoints.setInputCloud(cloud);
  SIFTPoints.compute(*siftcloud);

  pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
  pcl::visualization::PointCloudGeometryHandlerXYZ< PointWithScale > geometry(siftcloud); 
  viewer.addPointCloud(cloud, "cloud");
  viewer.addPointCloud(siftcloud, geometry, "keypoints");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "keypoints");
  viewer.spin();

  cout << "Size of siftcloud is " << siftcloud->points.size() << endl;

  // viewer.showCloud(siftcloud);
  while(!viewer.wasStopped())
    {

    }



  // // Compute the normals
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  // normal_estimation.setInputCloud (cloud);

  // pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
  // normal_estimation.setSearchMethod (kdtree);

  // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
  // normal_estimation.setRadiusSearch(2.0);
  // normal_estimation.compute (*normals);

  // // Setup spin image computation
  // pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > spin_image_descriptor(8, 0.5, 16);
  // spin_image_descriptor.setInputCloud (cloud);
  // spin_image_descriptor.setInputNormals (normals);

  // // Use the same KdTree from the normal estimation
  // spin_image_descriptor.setSearchMethod (kdtree);
  // pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
  // spin_image_descriptor.setRadiusSearch(2.0);

  // // Actually compute the spin images
  // spin_image_descriptor.compute (*spin_images);
  // std::cout << "SI output points.size (): " << spin_images->points.size () << std::endl;

  // // Display and retrieve the spin image descriptor vector for the first point.
  // pcl::Histogram<153> first_descriptor = spin_images->points[0];
  // cout << first_descriptor[1] << endl;

  return 0;
}
