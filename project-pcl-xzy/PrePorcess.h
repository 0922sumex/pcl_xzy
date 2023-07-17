#pragma once
#include <pcl/filters/voxel_grid.h>           //用于体素网格化的滤波类头文件 
#include <pcl/filters/filter.h>             //滤波相关头文件
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>          //滤波相关类头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计方法去除离群点
#include <pcl/filters/radius_outlier_removal.h> //统计方法去除离群点
#include <pcl/filters/approximate_voxel_grid.h>  //ApproximateVoxelGrid 


//点云下采样
void DownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
	//down sample
	std::cout << "begin downSample cloud_in size: " << cloud_in->size() << std::endl;
	pcl::VoxelGrid<pcl::PointXYZRGB> downSampled;  //创建滤波对象
	downSampled.setInputCloud(cloud_in);            //设置需要过滤的点云给滤波对象
	downSampled.setLeafSize(0.009f, 0.009f, 0.009f);  //设置滤波时创建的体素体积为1cm的立方体（1为米，0.01就是1cm）
	downSampled.filter(*cloud_out);  //执行滤波处理，存储输出

	std::cout << "success downSample, size: " << cloud_out->size() << std::endl;

}

//去除离群点
void OutlierFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
	std::cout << "begin outlierFilter cloud_in size: " << cloud_in->size() << std::endl;

	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> pcFilter;  //创建滤波器对象
	pcFilter.setInputCloud(cloud_in);             //设置待滤波的点云
	pcFilter.setRadiusSearch(0.03);               // 设置搜索半径
	pcFilter.setMinNeighborsInRadius(3);      // 设置一个内点最少的邻居数目
	pcFilter.filter(*cloud_out);        //滤波结果存储到cloud_filtered

	std::cout << "success OutlierFilter, size: " << cloud_out->size() << std::endl;

}


//对点云的预处理，就是点云下采样，去离群点滤波，
void PrePorcessingOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
	DownSample(cloud_in, cloud_temp);
	OutlierFilter(cloud_temp, cloud_out);
}
