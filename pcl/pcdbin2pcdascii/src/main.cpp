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

	if(pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		cout << "Couldn't read file" << endl;
		return -1;
	}

	if(pcl::io::savePCDFile(argv[2], *cloud, false) == -1)
	{
		cout << "Problem writing ASCII file. Not writing." << endl;
		return -2;
	}

	cout << "Done writing to file" << endl;

	return 0;
}

