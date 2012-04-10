#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

struct PointXYZRGBCamSL {
    PCL_ADD_POINT4D;

    union
	{
        struct
		{
            float rgb;
        };
        float data_c[4];
    };

    uint32_t cameraIndex;
    float distance;
    uint32_t segment;
    uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
        PointXYZRGBCamSL,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (uint32_t, cameraIndex, cameraIndex)
        (float, distance, distance)
        (uint32_t, segment, segment)
        (uint32_t, label, label)
        )

int main (int argc, char** argv)
{
	if(argc != 3)
	{
		cout << "Incorrect number of arguments. Usage: spinextract <Input PCD File> <Output ASCII Filename>" << endl;
		return -2;
	}

	pcl::PointCloud<PointXYZRGBCamSL>::Ptr cloud(new pcl::PointCloud<PointXYZRGBCamSL>);
//	pcl::PointCloud<PointXYZRGB>::Ptr cloud(new pcl::PointCloud<PointXYZRGB>);
//	sensor_msgs::PointCloud2 cloud_blob;

	if(pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file \n");
		return -1;
	}
//	pcl::fromROSMsg(cloud_blob, *cloud);

	if(pcl::io::savePCDFile(std::string("../data/ascii/")+std::string(argv[2]), *cloud, false) == -1) //* save the ascii file
	{
		cout << "Problem writing ASCII file. Not writing." << endl;
	}

//	cout << "Loaded "
//			<< cloud->width * cloud->height
//		    << " data points from .pcd file with the following fields: "
//		    << std::endl;

// CODE TO EXTRACT RGB
//	for(size_t i = 0; i < cloud->points.size(); ++i)
//	{
//		uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);

//		uint8_t r = (rgb >> 16) & 0x0000ff;
//		uint8_t g = (rgb >> 8)  & 0x0000ff;
//		uint8_t b = (rgb)       & 0x0000ff;

//		cout << +r << ", " << +g << ", " << +b << endl;
//	}

//	for(size_t i = 0; i < 10/*cloud->points.size()*/; ++i)
//	{
//		cout << "X: " << cloud->points[i].x << " Y: " << cloud->points[i].y << " Z: " << cloud->points[i].z << endl;
//	}

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while(!viewer.wasStopped())
	{

	}

	return 0;
}

