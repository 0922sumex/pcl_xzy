#include "plfh_solver.h"

void Calcu_angle(plane Plane_itself,plane_set PlaneSet) {
    //计算该平面到其他平面之间的角度

}

void Calcu_distance(plane Plane_itself,plane_set PlaneSet) {
    //计算该平面到其他平面之间的距离

}

void Calcu_PLFH(plane_set& PlaneSet)
{
    pcl::PointCloud<pcl::PLFH_scattered>::Ptr features(new pcl::PointCloud<pcl::PLFH_scattered>);

    //计算角度和距离
    for (int i = 0; i < PlaneSet.getPlaneNumber(); i++) {

        Calcu_angle(PlaneSet.getPlaneSet()[i], PlaneSet);
        Calcu_distance(PlaneSet.getPlaneSet()[i], PlaneSet);

    }
    // 计算法线
    /*pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(20); // 设置K近邻搜索参数
    ne.compute(*cloud_normals);*/

    // 计算特征描述符
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(cloud_normals);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr fpfh_tree(new pcl::search::KdTree<pcl::PointXYZ>());
    fpfh.setSearchMethod(fpfh_tree);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.setRadiusSearch(0.1); // 设置半径搜索参数
    fpfh.compute(*fpfhs);

    // 连接直方图
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_features(new pcl::PointCloud<pcl::PointXYZ>);
    combined_features->width = cloud->size();
    combined_features->height = 1;
    combined_features->points.resize(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        combined_features->points[i].x = fpfhs->points[i].histogram[0]; // 第一个直方图的第一个bin值
        combined_features->points[i].y = fpfhs->points[i].histogram[1]; // 第一个直方图的第二个bin值
        combined_features->points[i].z = fpfhs->points[i].histogram[2]; // 第一个直方图的第三个bin值

        // 这里假设第二个直方图的值是通过其他方式计算得到的
        // 请根据您的需求替换下面的代码
        // combined_features->points[i].z = ...;
    }

    // 保存特征描述符点云
    pcl::io::savePCDFileASCII("output_features.pcd", *combined_features);

    std::cout << "特征描述符计算完成并保存到output_features.pcd文件中。" << std::endl;

    return 0;
}

// 创建PFH估计类，并将输入数据集和法线传递给类
pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
pfh.setInputCloud(cloud);
pfh.setInputNormals(normals);
// 或者, 如果点云类型是 PointNormal, 使用pfh.setInputNormals (cloud);

// 创建一个空的kdtree表示，并将其传递给PFH对象。
// 树的内容将根据给定的输入数据集填充到对象内部(因为没有给出其他搜索面)。
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//PCL1.5以下版本使用这个指令
//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); 
pfh.setSearchMethod(tree);
// 输出PFH描述子
pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

// 使用半径为5cm的球面上的所有邻点
// 重要:这里使用的半径必须大于用来估计表面法线的半径！！！例如表面法向估计半径为0.02，此处为0.05
pfh.setRadiusSearch(0.05);

// 计算PFH描述子
pfh.compute(*pfhs);
}
