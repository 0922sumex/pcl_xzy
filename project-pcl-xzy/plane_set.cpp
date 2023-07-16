#include "plane_set.h"


#define MAX_ITERATION_    20


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
			//�����
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
	vector<int> inliers;				//�洢�ڵ�����������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_in(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_plane_out = cloud;

	Eigen::VectorXf coefficient_Judge(4);//�����ж�ƽ��ϵ���Ƿ��ظ�
	coefficient_Judge[0] = 0;
	coefficient_Judge[1] = 0;
	coefficient_Judge[2] = 0;
	coefficient_Judge[3] = 0;

	for (int i = 0; i < MAX_ITERATION_; i++) {
		//--------------------------RANSAC���ƽ��--------------------------
		pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_plane_out));
		pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_plane);
		ransac.setDistanceThreshold(0.01);	//���þ�����ֵ����ƽ�����С��0.01�ĵ���Ϊ�ڵ�
		ransac.computeModel();				//ִ��ģ�͹���

		//-------------------------����������ȡ�ڵ�--------------------------
		
		ransac.getInliers(inliers);			//��ȡ�ڵ�����
		pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, inliers, *cloud_plane_in);

		//----------------------------���ģ�Ͳ���---------------------------
		Eigen::VectorXf coefficient;
		ransac.getModelCoefficients(coefficient);
		if (coefficient_Judge == coefficient) {
			break;
		}
		coefficient_Judge = coefficient;
		plane plane_(coefficient, cloud_plane_in);//����ƽ��
		planeset.push_back(plane_);//����ƽ�漯
		plane_number++;//ƽ������һ
		//cout << "�ڵ����" << cloud_plane_in->size() << endl;
		cout << "ƽ�淽��Ϊ��\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + "
			<< coefficient[3] << " = 0" << endl;

		//-------------------------��ȡ�����ƣ���һʱ�̼�����ȡƽ��--------------------------
		//cout << "ƽ������" <<plane_.cloud_plane.size() << endl;
		copyOutPointCloud(*cloud, inliers, *cloud_plane_out);
		
	}
	
}

