#include "plane_set.h"


#define MAX_ITERATION_    100


void
copyOutPointCloud(const pcl::visualization::CloudViewer::MonochromeCloud& cloud_in,
	const pcl::Indices& indices,
	pcl::visualization::CloudViewer::MonochromeCloud& cloud_out)
{
	// Do we want to copy everything?
	if (indices.size() == cloud_in.points.size())
	{
		cloud_out = cloud_in;
		return;
	}

	// Allocate enough space and copy the basics
	cloud_out.points.resize(indices.size());
	cloud_out.header = cloud_in.header;
	cloud_out.width = static_cast<std::uint32_t>(indices.size());
	cloud_out.height = 1;
	cloud_out.is_dense = cloud_in.is_dense;
	cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
	cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

	// Iterate over each point
	std::size_t i=0,j = 0,t=0;
	for (; i < cloud_in.points.size()&& j < indices.size(); ++i,++j) {
		if (i != indices[j]) {
			//是外点
			cloud_out.points[t] = cloud_in.points[i];
		}
	}
	if (j >= indices.size() && i < cloud_in.points.size()) {
		for (; i < cloud_in.points.size(); i++) {
			cloud_out.points[t] = cloud_in.points[i];
		}
	}
}


void
plane_set::Extrace_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	vector<int> inliers;				//存储内点索引的容器
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_out(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_plane_out = cloud;
	/*Eigen::VectorXf coefficient_Judge;
	coefficient_Judge[0] = 0;
	coefficient_Judge[1] = 0;
	coefficient_Judge[2] = 0;
	coefficient_Judge[3] = 0;*/

	for (int i = 0; i < MAX_ITERATION_; i++) {
		//--------------------------RANSAC拟合平面--------------------------
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_plane_out));
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
		ransac.setDistanceThreshold(0.01);	//设置距离阈值，与平面距离小于0.01的点作为内点
		ransac.computeModel();				//执行模型估计

		//-------------------------根据索引提取内点--------------------------
		
		ransac.getInliers(inliers);			//提取内点索引
		pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_plane_in);

		//----------------------------输出模型参数---------------------------
		Eigen::VectorXf coefficient;
		ransac.getModelCoefficients(coefficient);
		/*if (coefficient_Judge == coefficient) {
			break;
		}
		coefficient_Judge = coefficient;*/

		plane plane_(coefficient,cloud_plane_in);//构建平面
		planeset.push_back(plane_);//加入平面集
		plane_number++;//平面数加一

		cout << "平面方程为：\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + "
			<< coefficient[3] << " = 0" << endl;

		//-------------------------提取外点点云，下一时刻继续提取平面--------------------------
		
		copyOutPointCloud(*cloud, inliers, *cloud_plane_out);
		
	}
	
}

