#include "Viewer_Results.h"
#include "PrePorcess.h"

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

	//结果可视化
	Viewer_Results_clouds(cloud_copy.makeShared(), PlaneSet_S);
}

