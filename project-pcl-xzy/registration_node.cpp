#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Viewer_Results.h"
#include "PrePorcess.h"
#include "plfh_solver.h"
#include "Bilateral_consensus.h"

int
main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//��ȡ�ĵ���
	pcl::PointCloud<pcl::PointXYZRGB> cloud_copy;//���Ƹ���
    plane_set PlaneSet_S;//Ms
    plane_set PlaneSet_T;//Mt
    vector<pcl::PLFH_gather> plfh_connected_set;//����PLFH
    pcl::PointXYZ point_single;
    
    LoadClouds(cloud);//��ȡ����
	cloud_copy = *cloud;
	PrePorcessingOfPointCloud(cloud_copy.makeShared(),cloud);//Ԥ����

	PlaneSet_S.Extrace_plane(cloud);//��ȡƽ��
    Viewer_Results_clouds(cloud_copy.makeShared(), PlaneSet_S);//���ӻ�����

    point_single = RandomSelectSingleCloud(cloud);//���ѡȡһ����
    Calcu_PLFHset(PlaneSet_S,plfh_connected_set,point_single);
    Viewer_Plotter(plfh_connected_set);//���ӻ�PLFH*/

    BilateralConsensus(plfh_connected_set, plfh_connected_set);
}

