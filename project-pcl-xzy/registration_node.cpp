#include <pcl/visualization/pcl_visualizer.h>
#include "plane_set.h"

#define MAX_ITERATION_    100

using namespace std;

int
main(int argc, char** argv)
{
    //����һ��������ƵĶ���
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //���ļ��м��ص���
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("ImageToStl.com_bun000.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from test_pcd.pcd with the following fields: "
        << std::endl;
    //��ȡƽ��
	plane_set PlaneSet_S;//Դ����ƽ�漯
	plane_set PlaneSet_T;//Ŀ�����ƽ�漯
	PlaneSet_S.Extrace_plane(cloud);
	vector<plane> cloud_plane_S = PlaneSet_S.getPlaneSet();
	
	//-----------------------------������ӻ�----------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("��Ͻ��"));

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	for (int i = 0; i < PlaneSet_S.getPlaneNumber(); i++) {

		viewer->addPointCloud<pcl::PointXYZ>(cloud_plane_S[i].cloud_plane, "plane");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "plane");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane");

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}
	}
	
	

	return 0;


    
}

