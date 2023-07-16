#pragma once
//利用RANSAC拟合得到多个平面

#include <pcl/sample_consensus/ransac.h>
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h> // 拟合平面
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

class plane {
public:
	//平面系数
	Eigen::VectorXf cofficient_set;
	//点云集
	pcl::PointCloud<pcl::PointXYZRGB> cloud_plane;
	//法向量

	//构造函数
	plane() {
		cofficient_set[0] = 0;
		cofficient_set[1] = 0;
		cofficient_set[2] = 0;
		cofficient_set[3] = 0;
	};
	plane(Eigen::VectorXf cofficientset, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_) {
		cofficient_set = cofficientset;
		cloud_plane = *cloud_plane_;
	}
};

class plane_set
{
public:
	
	//提取平面函数 最终得到一个平面集
	void Extrace_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	int getPlaneNumber() {
		return plane_number;
	};
	vector<plane> getPlaneSet() {
		return planeset;
	};
	plane_set() {
		plane_number = 0;
		planeset.clear();
	}
	
private:
	int plane_number;
	vector<plane> planeset;//平面集合

};

