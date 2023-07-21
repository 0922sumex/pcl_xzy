#include "Viewer_Results.h"
#include "PrePorcess.h"
#include "plfh_solver.h"
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main(int argc, char** argv)
{
    //����һ��������ƵĶ���
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> cloud_copy;

    //���ļ��м��ص���
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("cloud_bin_0.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from test_pcd.pcd with the following fields: "
        << std::endl;

	//Ԥ���� �²�����ȥ��Ⱥ��
	cloud_copy = *cloud;
	PrePorcessingOfPointCloud(cloud_copy.makeShared(),cloud);

    //��ȡƽ��
	plane_set PlaneSet_S;//Դ����ƽ�漯
	plane_set PlaneSet_T;//Ŀ�����ƽ�漯
	PlaneSet_S.Extrace_plane(cloud);
    Viewer_Results_clouds(cloud_copy.makeShared(), PlaneSet_S);//���ӻ�����

    vector<pcl::PLFH_gather> plfh_connected_set;//����PLFH
    pcl::PointXYZ point_single;
    int j = 0;
    point_single.x = cloud->points[j].x;
    point_single.y = cloud->points[j].y;
    point_single.z = cloud->points[j].z;//���ѡȡһ����

    Calcu_PLFHset(PlaneSet_S,plfh_connected_set,point_single);
    Viewer_Plotter(plfh_connected_set);//���ӻ�PLFH
}

