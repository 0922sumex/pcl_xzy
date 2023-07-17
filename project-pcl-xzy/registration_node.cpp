#include "Viewer_Results.h"
#include "PrePorcess.h"

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

	//������ӻ�
	Viewer_Results_clouds(cloud_copy.makeShared(), PlaneSet_S);
}

