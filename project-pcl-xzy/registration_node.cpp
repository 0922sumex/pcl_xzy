#include "Viewer_Results.h"
#include "PrePorcess.h"
#include "plfh_solver.h"
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main(int argc, char** argv)
{
    //创建一个保存点云的对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> cloud_copy;

    //从文件中加载点云
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("cloud_bin_0.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from test_pcd.pcd with the following fields: "
        << std::endl;

	//预处理 下采样及去离群点
	cloud_copy = *cloud;
	PrePorcessingOfPointCloud(cloud_copy.makeShared(),cloud);

    //提取平面
	plane_set PlaneSet_S;//源点云平面集
	plane_set PlaneSet_T;//目标点云平面集
	PlaneSet_S.Extrace_plane(cloud);
    Viewer_Results_clouds(cloud_copy.makeShared(), PlaneSet_S);//可视化点云

    vector<pcl::PLFH_gather> plfh_connected_set;//计算PLFH
    pcl::PointXYZ point_single;
    int j = 0;
    point_single.x = cloud->points[j].x;
    point_single.y = cloud->points[j].y;
    point_single.z = cloud->points[j].z;//随机选取一个点

    Calcu_PLFHset(PlaneSet_S,plfh_connected_set,point_single);
    Viewer_Plotter(plfh_connected_set);//可视化PLFH
}

