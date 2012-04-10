#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

#include <pcl/visualization/cloud_viewer.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  int n;
  char buffer [50];

  for (int INDscene = 3; INDscene <= 3; ++INDscene ) // for all scenes
  {
    //n=sprintf (buffer, "../../pcd_write/build/test_pcd.pcd");
    //n=sprintf (buffer, "/opt/ros/electric/stacks/perception_pcl_addons/pcl_visualization/data/partial_cup_model.pcd"); 
    //n=sprintf (buffer, "../../home_data/original/scene%d.pcd", INDscene);
//    n=sprintf (buffer, "/media/TEMPSTORAGE/545ProjectData/others/partial_cup_model.pcd", INDscene);
    //n=sprintf (buffer, "../../home_data/converted_ascii/scene%d.pcd", INDscene);   

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/media/TEMPSTORAGE/545ProjectData/others/office_scene.pcd", *cloud) == -1) {} //* load the file
    else
    {
      std::cout << "Reading file " << buffer << "...  " << std::endl;
      std::cout << "Width " << cloud->width << " Height " << cloud->height << std::endl;
      
      // Compute the normals
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
      normal_estimation.setInputCloud (cloud);

      pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
      normal_estimation.setSearchMethod (kdtree);
      
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
      normal_estimation.setRadiusSearch (0.03);
      normal_estimation.compute (*normals);
      
      // Visualizing normals
      pcl::visualization::PCLVisualizer viewer("PCL Viewer");
      viewer.setBackgroundColor (0.0, 0.0, 0.5);
      viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);

      // // Setup spin image computation
      // pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > spin_image_descriptor(8, 0.5, 16);
      // spin_image_descriptor.setInputCloud (cloud);
      // spin_image_descriptor.setInputNormals (normals);
      
      // // Use the same KdTree from the normal estimation
      // spin_image_descriptor.setSearchMethod (kdtree);
      // pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
      // spin_image_descriptor.setRadiusSearch (0.2);
      
      // // Actually compute the spin images
      // spin_image_descriptor.compute (*spin_images);
      // std::cout << "SI output points.size (): " << spin_images->points.size () << std::endl;
      
      // // Display and retrieve the spin image descriptor vector for the first point.
      // pcl::Histogram<153> first_descriptor = spin_images->points[0];
      // std::cout << first_descriptor << std::endl;
      
      


      // for (size_t i = 0; i < cloud->points.size (); ++i) // for all points
      // {	
      // 	std::cout << "ind " << i
      //             << " X " << cloud->points[i].x 
      // 		  << " Y " << cloud->points[i].y
      // 		  << " Z " << cloud->points[i].z << std::endl;
      // }

    }
  }
  
  return 0;

}
