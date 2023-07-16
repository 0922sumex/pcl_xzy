#pragma once
#include <pcl/filters/voxel_grid.h>           //�����������񻯵��˲���ͷ�ļ� 
#include <pcl/filters/filter.h>             //�˲����ͷ�ļ�
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>          //�˲������ͷ�ļ�
#include <pcl/filters/statistical_outlier_removal.h> //ͳ�Ʒ���ȥ����Ⱥ��
#include <pcl/filters/radius_outlier_removal.h> //ͳ�Ʒ���ȥ����Ⱥ��
#include <pcl/filters/approximate_voxel_grid.h>  //ApproximateVoxelGrid 


//�����²���
void DownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
	//down sample
	std::cout << "begin downSample cloud_in size: " << cloud_in->size() << std::endl;
	pcl::VoxelGrid<pcl::PointXYZRGB> downSampled;  //�����˲�����
	downSampled.setInputCloud(cloud_in);            //������Ҫ���˵ĵ��Ƹ��˲�����
	downSampled.setLeafSize(0.01f, 0.01f, 0.01f);  //�����˲�ʱ�������������Ϊ1cm�������壨1Ϊ�ף�0.01����1cm��
	downSampled.filter(*cloud_out);  //ִ���˲������洢���

	std::cout << "success downSample, size: " << cloud_out->size() << std::endl;

}

//ȥ����Ⱥ��
void OutlierFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
	std::cout << "begin outlierFilter cloud_in size: " << cloud_in->size() << std::endl;

	//ͳ�ƣ��е���
		//pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //�����˲�������
		//sor.setInputCloud(cloud_in);                           //���ô��˲��ĵ���
		//sor.setMeanK(50);                               //�����ڽ���ͳ��ʱ���ǵ��ٽ������
		//sor.setStddevMulThresh(1.0);                      //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ���������˱�׼��
		//sor.filter(*cloud_out);                    //�˲�����洢��cloud_filtered
		//

	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> pcFilter;  //�����˲�������
	pcFilter.setInputCloud(cloud_in);             //���ô��˲��ĵ���
	pcFilter.setRadiusSearch(0.03);               // ���������뾶
	pcFilter.setMinNeighborsInRadius(3);      // ����һ���ڵ����ٵ��ھ���Ŀ
	pcFilter.filter(*cloud_out);        //�˲�����洢��cloud_filtered

	std::cout << "success OutlierFilter, size: " << cloud_out->size() << std::endl;

}


//�Ե��Ƶ�Ԥ�������ǵ����²�����ȥ��Ⱥ���˲���
void PrePorcessingOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
	//pcl::StopWatch time;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());

	DownSample(cloud_in, cloud_temp);

	OutlierFilter(cloud_temp, cloud_out);

	//std::cout << "point cloud pre processing time(s): " << time.getTimeSeconds() << std::endl;
}
