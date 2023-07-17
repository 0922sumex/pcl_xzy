#include "plane_set.h"


#define MAX_ITERATION_    10

//vector<int> inliers;				//�洢�ڵ�����������
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//�����б�
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_out(new pcl::PointCloud<pcl::PointXYZ>);
pcl::SACSegmentation<pcl::PointXYZ> seg;//�ָ����
pcl::ExtractIndices<pcl::PointXYZ> extract;//��ȡ��
pcl::ModelCoefficients::Ptr cofficient(new pcl::ModelCoefficients);//ģ��ϵ��
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
	pcl::copyPointCloud(*cloud, *cloud_plane_out);
	//cloud_plane_out = cloud;
	pcl::ModelCoefficients coefficient_Judge;//�����ж�ƽ��ϵ���Ƿ��ظ�
	coefficient_Judge.values.clear();

	for (int i = 0; i < MAX_ITERATION_; i++) {
		//RANSAC���ƽ��
		/*pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_plane_out));
		pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_plane);
		ransac.setDistanceThreshold(0.07);	//���þ�����ֵ����ƽ�����С��0.01�ĵ���Ϊ�ڵ�
		ransac.computeModel();				//ִ��ģ�͹���*/
		seg.setOptimizeCoefficients(true);		//ʹ���ڲ������¹���ģ�Ͳ���
		seg.setModelType(pcl::SACMODEL_PLANE);   //����ģ������
		seg.setMethodType(pcl::SAC_RANSAC);      //�����������һ���Է�������
		seg.setDistanceThreshold(0.01);          //�趨���뷧ֵ�����뷧ֵ�����˵㱻��Ϊ�Ǿ��ڵ��Ǳ������������
		seg.setInputCloud(cloud_plane_out);
		seg.segment(*inliers, *cofficient);
		if (coefficient_Judge.values == (*cofficient).values) {
			break;
		}
		coefficient_Judge = *cofficient;


		extract.setInputCloud(cloud_plane_out);
		extract.setIndices(inliers);
		extract.setNegative(false);
		//��ȡ̽�������ƽ��
		extract.filter(*cloud_plane_in);
		//cloud_plane_inΪ�ô�̽���������Ƭ�����Ե������б���

		//�޳�̽�����ƽ�棬��ʣ����м���̽��ƽ��
		extract.setNegative(true);
		extract.filter(*cloud_plane_out);

		//�洢
		plane plane_(*cofficient, cloud_plane_in);//����ƽ��
		planeset.push_back(plane_);//����ƽ�漯
		plane_number++;//ƽ������һ
		cout << "ƽ�淽��Ϊ��\n" << cofficient->values[0] << "x + " << cofficient->values[1] << "y + " << cofficient->values[2] << "z + "
			<< cofficient->values[3] << " = 0" << endl;
		//����������ȡ�ڵ�
		/*ransac.getInliers(inliers);			//��ȡ�ڵ�����
		pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, inliers, *cloud_plane_in);

		//���ģ�Ͳ���
		ransac.getModelCoefficients(coefficient);
		if (coefficient_Judge == coefficient) {
			break;
		}
		coefficient_Judge = coefficient;

		//�÷��������ж��Ƿ���Ժ�����ƽ�����һ��
		//

		plane plane_(coefficient, cloud_plane_in);//����ƽ��
		planeset.push_back(plane_);//����ƽ�漯
		plane_number++;//ƽ������һ
		cout << "ƽ�淽��Ϊ��\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + "
			<< coefficient[3] << " = 0" << endl;

		//��ȡ�����ƣ���һʱ�̼�����ȡƽ��
		copyOutPointCloud(*cloud, inliers, *cloud_plane_out);
		*/
	}
	
}

