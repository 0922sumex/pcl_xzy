#include "plane_set.h"

#define MAX_ITERATION_    10

pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//�����ڵ������
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_segment(new pcl::PointCloud<pcl::PointXYZ>);//�ָ����ƽ��
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::SACSegmentation<pcl::PointXYZ> seg;//�ָ����
pcl::ExtractIndices<pcl::PointXYZ> extract;//��ȡ��
pcl::ModelCoefficients::Ptr cofficient(new pcl::ModelCoefficients);//ģ��ϵ��

void
plane_set::Extrace_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::copyPointCloud(*cloud, *cloud_plane_in);
	pcl::ModelCoefficients coefficient_Judge;//�����ж�ƽ��ϵ���Ƿ��ظ�
	coefficient_Judge.values.clear();

	for (int i = 0; i < MAX_ITERATION_; i++) {
		//���ƽ��
		seg.setOptimizeCoefficients(true);		//ʹ���ڲ������¹���ģ�Ͳ���
		seg.setModelType(pcl::SACMODEL_PLANE);   //����ģ������
		seg.setMethodType(pcl::SAC_RANSAC);      //�����������һ���Է�������
		seg.setDistanceThreshold(0.01);          //�趨���뷧ֵ������ƽ���ڵ�
		seg.setInputCloud(cloud_plane_in);
		seg.segment(*inliers, *cofficient);
		if (coefficient_Judge.values == (*cofficient).values) {
			break;
		}//ǰ������ϵ����ͬ��ѭ��ֹͣ
		coefficient_Judge = *cofficient;

		//��ȡ̽�������ƽ��
		extract.setInputCloud(cloud_plane_in);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*plane_segment);

		//�޳�̽�����ƽ�棬��ʣ����м���̽��ƽ��
		extract.setNegative(true);
		extract.filter(*cloud_plane_in);

		//�洢
		plane plane_(*cofficient, plane_segment);//����ƽ��
		planeset.push_back(plane_);//����ƽ�漯
		plane_number++;//ƽ������һ
		cout << "ƽ�淽��Ϊ��\n" << cofficient->values[0] << "x + " << cofficient->values[1] << "y + " << cofficient->values[2] << "z + "
			<< cofficient->values[3] << " = 0" << endl;
	}
	
}

