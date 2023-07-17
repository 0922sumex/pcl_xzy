#pragma once
//����RANSAC��ϵõ����ƽ��

#include <pcl/sample_consensus/ransac.h>
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>   //RANSAC�ָ�
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h> // ���ƽ��
#include <pcl/sample_consensus/method_types.h>   //����������Ʒ���
#include <pcl/sample_consensus/model_types.h>    //ģ�Ͷ���

using namespace std;

typedef struct normal {
	float normal_x;
	float normal_y;
	float normal_z;
}Normal;

class plane {
public:
	//ƽ��ϵ��
	pcl::ModelCoefficients cofficient_set;//ģ��ϵ��
	pcl::PointCloud<pcl::PointXYZRGB> cloud_plane;//���Ƽ�
	Normal normal;
	
	//���캯��
	plane() {
		
	};
	plane(pcl::ModelCoefficients cofficientset, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_) {
		cofficient_set = cofficientset;
		//cloud_plane = *cloud_plane_;
		pcl::copyPointCloud(*cloud_plane_, cloud_plane);
		normal.normal_x = cofficient_set.values[0];
		normal.normal_y = cofficient_set.values[1];
		normal.normal_z = cofficient_set.values[2];
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

