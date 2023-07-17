#include "plane_set.h"


#define MAX_ITERATION_    10

//vector<int> inliers;				//存储内点索引的容器
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//索引列表
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_out(new pcl::PointCloud<pcl::PointXYZ>);
pcl::SACSegmentation<pcl::PointXYZ> seg;//分割对象
pcl::ExtractIndices<pcl::PointXYZ> extract;//提取器
pcl::ModelCoefficients::Ptr cofficient(new pcl::ModelCoefficients);//模型系数
//Eigen::VectorXf coefficient;

void
copyOutPointCloud(const pcl::visualization::CloudViewer::ColorCloud& cloud_in,
	const pcl::Indices& indices,
	pcl::visualization::CloudViewer::ColorCloud& cloud_out)
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
plane_set::Extrace_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::copyPointCloud(*cloud, *cloud_plane_out);
	//cloud_plane_out = cloud;
	pcl::ModelCoefficients coefficient_Judge;//用来判断平面系数是否重复
	coefficient_Judge.values.clear();

	for (int i = 0; i < MAX_ITERATION_; i++) {
		//RANSAC拟合平面
		/*pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_plane_out));
		pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_plane);
		ransac.setDistanceThreshold(0.07);	//设置距离阈值，与平面距离小于0.01的点作为内点
		ransac.computeModel();				//执行模型估计*/
		seg.setOptimizeCoefficients(true);		//使用内部点重新估算模型参数
		seg.setModelType(pcl::SACMODEL_PLANE);   //设置模型类型
		seg.setMethodType(pcl::SAC_RANSAC);      //设置随机采样一致性方法类型
		seg.setDistanceThreshold(0.01);          //设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
		seg.setInputCloud(cloud_plane_out);
		seg.segment(*inliers, *cofficient);
		if (coefficient_Judge.values == (*cofficient).values) {
			break;
		}
		coefficient_Judge = *cofficient;


		extract.setInputCloud(cloud_plane_out);
		extract.setIndices(inliers);
		extract.setNegative(false);
		//提取探测出来的平面
		extract.filter(*cloud_plane_in);
		//cloud_plane_in为该次探测出来的面片，可以单独进行保存

		//剔除探测出的平面，在剩余点中继续探测平面
		extract.setNegative(true);
		extract.filter(*cloud_plane_out);

		//存储
		plane plane_(*cofficient, cloud_plane_in);//构建平面
		planeset.push_back(plane_);//加入平面集
		plane_number++;//平面数加一
		cout << "平面方程为：\n" << cofficient->values[0] << "x + " << cofficient->values[1] << "y + " << cofficient->values[2] << "z + "
			<< cofficient->values[3] << " = 0" << endl;
		//根据索引提取内点
		/*ransac.getInliers(inliers);			//提取内点索引
		pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, inliers, *cloud_plane_in);

		//输出模型参数
		ransac.getModelCoefficients(coefficient);
		if (coefficient_Judge == coefficient) {
			break;
		}
		coefficient_Judge = coefficient;

		//用法向量来判断是否可以和其他平面合在一起
		//

		plane plane_(coefficient, cloud_plane_in);//构建平面
		planeset.push_back(plane_);//加入平面集
		plane_number++;//平面数加一
		cout << "平面方程为：\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + "
			<< coefficient[3] << " = 0" << endl;

		//提取外点点云，下一时刻继续提取平面
		copyOutPointCloud(*cloud, inliers, *cloud_plane_out);
		*/
	}
	
}

