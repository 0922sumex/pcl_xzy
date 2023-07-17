#include "plfh_solver.h"

void Calcu_angle(plane Plane_itself,plane_set PlaneSet) {
    //�����ƽ�浽����ƽ��֮��ĽǶ�

}

void Calcu_distance(plane Plane_itself,plane_set PlaneSet) {
    //�����ƽ�浽����ƽ��֮��ľ���

}

void Calcu_PLFH(plane_set& PlaneSet)
{
    pcl::PointCloud<pcl::PLFH_scattered>::Ptr features(new pcl::PointCloud<pcl::PLFH_scattered>);

    //����ǶȺ;���
    for (int i = 0; i < PlaneSet.getPlaneNumber(); i++) {

        Calcu_angle(PlaneSet.getPlaneSet()[i], PlaneSet);
        Calcu_distance(PlaneSet.getPlaneSet()[i], PlaneSet);

    }
    // ���㷨��
    /*pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(20); // ����K������������
    ne.compute(*cloud_normals);*/

    // ��������������
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(cloud_normals);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr fpfh_tree(new pcl::search::KdTree<pcl::PointXYZ>());
    fpfh.setSearchMethod(fpfh_tree);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.setRadiusSearch(0.1); // ���ð뾶��������
    fpfh.compute(*fpfhs);

    // ����ֱ��ͼ
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_features(new pcl::PointCloud<pcl::PointXYZ>);
    combined_features->width = cloud->size();
    combined_features->height = 1;
    combined_features->points.resize(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        combined_features->points[i].x = fpfhs->points[i].histogram[0]; // ��һ��ֱ��ͼ�ĵ�һ��binֵ
        combined_features->points[i].y = fpfhs->points[i].histogram[1]; // ��һ��ֱ��ͼ�ĵڶ���binֵ
        combined_features->points[i].z = fpfhs->points[i].histogram[2]; // ��һ��ֱ��ͼ�ĵ�����binֵ

        // �������ڶ���ֱ��ͼ��ֵ��ͨ��������ʽ����õ���
        // ��������������滻����Ĵ���
        // combined_features->points[i].z = ...;
    }

    // ������������������
    pcl::io::savePCDFileASCII("output_features.pcd", *combined_features);

    std::cout << "����������������ɲ����浽output_features.pcd�ļ��С�" << std::endl;

    return 0;
}

// ����PFH�����࣬�����������ݼ��ͷ��ߴ��ݸ���
pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
pfh.setInputCloud(cloud);
pfh.setInputNormals(normals);
// ����, ������������� PointNormal, ʹ��pfh.setInputNormals (cloud);

// ����һ���յ�kdtree��ʾ�������䴫�ݸ�PFH����
// �������ݽ����ݸ������������ݼ���䵽�����ڲ�(��Ϊû�и�������������)��
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//PCL1.5���°汾ʹ�����ָ��
//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); 
pfh.setSearchMethod(tree);
// ���PFH������
pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

// ʹ�ð뾶Ϊ5cm�������ϵ������ڵ�
// ��Ҫ:����ʹ�õİ뾶��������������Ʊ��淨�ߵİ뾶������������淨����ư뾶Ϊ0.02���˴�Ϊ0.05
pfh.setRadiusSearch(0.05);

// ����PFH������
pfh.compute(*pfhs);
}
