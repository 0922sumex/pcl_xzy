#include <pcl/visualization/pcl_visualizer.h>
#include <random>
#include "plane_set.h"
#include "PrePorcess.h"

using namespace std;
using namespace pcl;

// 生成随机颜色
pcl::RGB generateRandomColor()
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(0, 255);

	pcl::RGB color;
	color.r = static_cast<uint8_t>(dis(gen));
	color.g = static_cast<uint8_t>(dis(gen));
	color.b = static_cast<uint8_t>(dis(gen));

	return color;
}

int
main(int argc, char** argv)
{
    //创建一个保存点云的对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> cloud_3;
    //从文件中加载点云
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("cloud_bin_1.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from test_pcd.pcd with the following fields: "
        << std::endl;

	//预处理 下采样及去离群点
	cloud_3 = *cloud;
	PrePorcessingOfPointCloud(cloud_3.makeShared(),cloud);

    //提取平面
	plane_set PlaneSet_S;//源点云平面集
	plane_set PlaneSet_T;//目标点云平面集

	PlaneSet_S.Extrace_plane(cloud);
	vector<plane> cloud_plane_S = PlaneSet_S.getPlaneSet();
	
	//-----------------------------结果可视化----------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("拟合结果"));
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_3.makeShared(), "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	cout << "平面个数为：" << PlaneSet_S.getPlaneNumber() << endl;

	for (int i = 0; i < PlaneSet_S.getPlaneNumber(); i++) {
		//生成随机颜色
		pcl::RGB color1=generateRandomColor();
		cout << "输出第i个平面的点云" << i << endl;

		for (auto& point : cloud_plane_S[i].cloud_plane.points)
		{
			point.r = color1.r;
			point.g = color1.g;
			point.b = color1.b;
		}
		viewer->removePointCloud(std::to_string(i));
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud_plane_S[i].cloud_plane.makeShared(), std::to_string(i));
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, std::to_string(i));
	}
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}

