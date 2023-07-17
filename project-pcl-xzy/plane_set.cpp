#include "plane_set.h"

#define MAX_ITERATION_    10

pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//索引内点的容器
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_segment(new pcl::PointCloud<pcl::PointXYZ>);//分割出的平面
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::SACSegmentation<pcl::PointXYZ> seg;//分割对象
pcl::ExtractIndices<pcl::PointXYZ> extract;//提取器
pcl::ModelCoefficients::Ptr cofficient(new pcl::ModelCoefficients);//模型系数

void
plane_set::Extrace_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::copyPointCloud(*cloud, *cloud_plane_in);
	pcl::ModelCoefficients coefficient_Judge;//用来判断平面系数是否重复
	coefficient_Judge.values.clear();

	for (int i = 0; i < MAX_ITERATION_; i++) {
		//拟合平面
		seg.setOptimizeCoefficients(true);		//使用内部点重新估算模型参数
		seg.setModelType(pcl::SACMODEL_PLANE);   //设置模型类型
		seg.setMethodType(pcl::SAC_RANSAC);      //设置随机采样一致性方法类型
		seg.setDistanceThreshold(0.01);          //设定距离阀值来决定平面内点
		seg.setInputCloud(cloud_plane_in);
		seg.segment(*inliers, *cofficient);
		if (coefficient_Judge.values == (*cofficient).values) {
			break;
		}//前后两次系数相同则循环停止
		coefficient_Judge = *cofficient;

		//提取探测出来的平面
		extract.setInputCloud(cloud_plane_in);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*plane_segment);

		//剔除探测出的平面，在剩余点中继续探测平面
		extract.setNegative(true);
		extract.filter(*cloud_plane_in);

		//存储
		plane plane_(*cofficient, plane_segment);//构建平面
		planeset.push_back(plane_);//加入平面集
		plane_number++;//平面数加一
		cout << "平面方程为：\n" << cofficient->values[0] << "x + " << cofficient->values[1] << "y + " << cofficient->values[2] << "z + "
			<< cofficient->values[3] << " = 0" << endl;
	}
	
}

