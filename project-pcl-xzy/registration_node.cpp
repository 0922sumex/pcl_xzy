#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Viewer_Results.h"
#include "PrePorcess.h"
#include "plfh_solver.h"
#include "Bilateral_consensus.h"

int
main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//读取的点云
	pcl::PointCloud<pcl::PointXYZRGB> cloud_copy;//点云副本
    plane_set PlaneSet_S;//Ms
    plane_set PlaneSet_T;//Mt
    vector<pcl::PLFH_gather> plfh_connected_set;//计算PLFH
    pcl::PointXYZ point_single;
    
    LoadClouds(cloud);//读取点云
	cloud_copy = *cloud;
	PrePorcessingOfPointCloud(cloud_copy.makeShared(),cloud);//预处理

	PlaneSet_S.Extrace_plane(cloud);//提取平面
    Viewer_Results_clouds(cloud_copy.makeShared(), PlaneSet_S);//可视化点云

    point_single = RandomSelectSingleCloud(cloud);//随机选取一个点
    Calcu_PLFHset(PlaneSet_S,plfh_connected_set,point_single);
    Viewer_Plotter(plfh_connected_set);//可视化PLFH*/

    BilateralConsensus(plfh_connected_set, plfh_connected_set);
}

