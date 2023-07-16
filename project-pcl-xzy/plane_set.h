#pragma once
//����RANSAC��ϵõ����ƽ��

#include <pcl/sample_consensus/ransac.h>
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h> // ���ƽ��
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

class plane {
public:
	//ƽ��ϵ��
	Eigen::VectorXf cofficient_set;
	//���Ƽ�
	pcl::PointCloud<pcl::PointXYZRGB> cloud_plane;
	//������

	//���캯��
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
	
	//��ȡƽ�溯�� ���յõ�һ��ƽ�漯
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
	vector<plane> planeset;//ƽ�漯��

};

