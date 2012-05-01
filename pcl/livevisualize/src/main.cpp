#include <iostream>
#include <vector>
#include <map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

#include "extern.hpp"

using namespace std;
using namespace pcl;
using namespace boost;

PointCloud<PointXYZRGB >::Ptr cloud(new PointCloud<PointXYZRGB >);


bool ParseFiles(void)
{
//	ifstream LabFile("result_live_rbmsvm.txt");
	ifstream LabFile("result_live_dbn.txt");
	map<int, int> labels;
	while (!LabFile.eof())
	{
		int s, l;
		LabFile >> s >> l;
		labels[s] = l;
	}


	ifstream SegFile("live_segments.txt");

	string SegLabels;
	int position = 0;
	while (!SegFile.eof())
	{
		getline(SegFile, SegLabels);
		char_separator<char > sep(" ");	    
		string SegIdx;
		getline(SegFile, SegIdx);
		tokenizer<char_separator<char > > tokens2(SegIdx, sep);

		BOOST_FOREACH(string t, tokens2)
		{
			int idx = atoi(t.c_str());
			//cout << idx << endl;
			//cout << labels[position] << endl;
			switch(labels[position])
			{
				case 1:
					cloud->points.at(idx).r = uint8_t(240);
					cloud->points.at(idx).g = uint8_t(248);
					cloud->points.at(idx).b = uint8_t(255);
					break;
				case 2:
					cloud->points.at(idx).r = uint8_t(25);
					cloud->points.at(idx).g = uint8_t(25);
					cloud->points.at(idx).b = uint8_t(112);
					break;
				case 3:
					cloud->points.at(idx).r = uint8_t(135);
					cloud->points.at(idx).g = uint8_t(206);
					cloud->points.at(idx).b = uint8_t(250);
					break;
				case 4:
					cloud->points.at(idx).r = uint8_t(205);
					cloud->points.at(idx).g = uint8_t(92);
					cloud->points.at(idx).b = uint8_t(92);
					break;
				case 5:
					cloud->points.at(idx).r = uint8_t(255);
					cloud->points.at(idx).g = uint8_t(0);
					cloud->points.at(idx).b = uint8_t(0);
					break;
				case 6:
					cloud->points.at(idx).r = uint8_t(0);
					cloud->points.at(idx).g = uint8_t(255);
					cloud->points.at(idx).b = uint8_t(0);
					break;
				case 7:
					cloud->points.at(idx).r = uint8_t(0);
					cloud->points.at(idx).g = uint8_t(0);
					cloud->points.at(idx).b = uint8_t(255);
					break;
				case 8:
					cloud->points.at(idx).r = uint8_t(255);
					cloud->points.at(idx).g = uint8_t(105);
					cloud->points.at(idx).b = uint8_t(180);
					break;
				case 9:
					cloud->points.at(idx).r = uint8_t(208);
					cloud->points.at(idx).g = uint8_t(32);
					cloud->points.at(idx).b = uint8_t(144);
					break;
				case 10:
					cloud->points.at(idx).r = uint8_t(0);
					cloud->points.at(idx).g = uint8_t(255);
					cloud->points.at(idx).b = uint8_t(255);
					break;
				case 11:
					cloud->points.at(idx).r = uint8_t(238);
					cloud->points.at(idx).g = uint8_t(0);
					cloud->points.at(idx).b = uint8_t(0);
					break;
				default:
					cloud->points.at(idx).r = uint8_t(190);
					cloud->points.at(idx).g = uint8_t(0);
					cloud->points.at(idx).b = uint8_t(0);				
					break;
			}
		}
		
		position++;
	}

	SegFile.close();
	LabFile.close();
}

int main(int argc, char** argv)
{
	if(pcl::io::loadPCDFile<PointXYZRGB >("live.pcd", *cloud) == -1)
	{
		cout << "Couldn't read some files. Must be run in sequence." << endl;
		return -1;
	}

	ParseFiles();

	pcl::visualization::CloudViewer viewer("Classified Output");
	viewer.showCloud(cloud);

	while(!viewer.wasStopped())
	{

	}

	return 0;
}

