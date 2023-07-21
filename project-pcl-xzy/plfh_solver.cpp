#include "plfh_solver.h"


void calcLine(pcl::ModelCoefficients coefsOfPlane1, pcl::ModelCoefficients coefsOfPlane2, LinePara3D& coefsOfLine)
{
    //算出两平面间的交线
    double a1, b1, c1, d1, a2, b2, c2, d2;
    double tempy, tempz;
    a1 = coefsOfPlane1.values[0];
    b1 = coefsOfPlane1.values[1];
    c1 = coefsOfPlane1.values[2];
    d1 = coefsOfPlane1.values[3];
    a2 = coefsOfPlane2.values[0];
    b2 = coefsOfPlane2.values[1];
    c2 = coefsOfPlane2.values[2];
    d2 = coefsOfPlane2.values[3];
    tempz = -(d1 / b1 - d2 / b2) / (c1 / b1 - c2 / b2);
    tempy = (-c1 / b1) * tempz - d1 / b1;
    //直线上一点
    point_line point_temp;
    point_temp.x = 0.0;
    point_temp.y = tempy;
    point_temp.z = tempz;
    coefsOfLine.point_several.push_back(point_temp);//交线的一个点（0,y,z）
    //交线的方向向量
    coefsOfLine.line_direction.dierctionlX=b1 * c2 - c1 * b2;
    coefsOfLine.line_direction.dierctionlY=c1 * a2 - a1 * c2;
    coefsOfLine.line_direction.dierctionlZ=a1 * b2 - b1 * a2;

    //交线的法向量  (b, -a, 0) 
    coefsOfLine.line_normal.normalX= c1 * a2 - a1 * c2;
    coefsOfLine.line_normal.normalY = c1 * b2 - b1 * c2;
    coefsOfLine.line_normal.normalZ = 0;
}


double Point2Line3DVecproduct(LinePara3D line, pcl::PointXYZ point)
{
    //计算固定点到交线的距离
    //直线的法向量(p,q,r)
    double p = line.line_normal.normalX;
    double q = line.line_normal.normalY;
    double r = line.line_normal.normalZ;

    //直线上的2个点q和b
    double x_q = line.point_several[0].x;
    double y_q = line.point_several[0].y;
    double z_q = line.point_several[0].z;

    double x_b = 1; //令第2个点的x=1
    double y_b = y_q - x_q * q / p + q / p;
    double z_b = z_q - x_q * r / p + r / p;

    double x_o = point.x;
    double y_o = point.y;
    double z_o = point.z;

    //两个向量
    //oq
    double normal01_x = x_q - x_o;
    double normal01_y = y_q - y_o;
    double normal01_z = z_q - z_o;

    //ob
    double normal02_x = x_b - x_o;
    double normal02_y = y_b - y_o;
    double normal02_z = z_b - z_o;

    //oq与ob进行叉乘
    double chacheng_x = normal01_y * normal02_z - normal02_y * normal01_z;
    double chacheng_y = normal02_z * normal01_x - normal02_x * normal01_z;
    double chacheng_z = normal01_x * normal02_y - normal01_y * normal02_x;

    double chacheng_length = sqrt(chacheng_x * chacheng_x + chacheng_y * chacheng_y + chacheng_z * chacheng_z);
    double dx = x_q - x_b;
    double dy = y_q - y_b;
    double dz = z_q - z_b;

    double qb_length = sqrt(dx * dx + dy * dy + dz * dz);

    double ds = chacheng_length / qb_length;

    //cout << "算出固定点到平面间交线的距离：" << ds << endl;

    return ds;

}

pcl::PLFH_scattered Calcu_plfh (int index,plane_set PlaneSet, pcl::PointXYZ point) {

    //计算该平面到其他平面之间的角度和距离

    pcl::PLFH_scattered plfh;
    plane Plane_itself=PlaneSet.getPlaneSet()[index];
    Eigen::Vector3f n1 = Plane_itself.normal;
    LinePara3D coefsOfLine;//相交线 

    for (int i = 0; i < PlaneSet.getPlaneNumber(); i++) {
        if (i == index) {
            continue;//本身不参与运算
        }
        Eigen::Vector3f n2 = PlaneSet.getPlaneSet()[i].normal;
        // 归一化向量
        n1.normalize();
        n2.normalize();
        float dotProduct = n1.dot(n2);
        float n1Norm = n1.norm();
        float n2Norm = n2.norm();

        // 使用反余弦函数计算夹角
        float angle = std::acos(dotProduct / (n1Norm * n2Norm));
        plfh.angle_histogram.push_back(angle);
        plfh.nr_dimensions_++;

        // 计算一个固定点到平面相交线之间的距离
        LinePara3D coefsOfLine;
        calcLine(Plane_itself.cofficient_set,PlaneSet.getPlaneSet()[i].cofficient_set,coefsOfLine);
        double dis_line=Point2Line3DVecproduct(coefsOfLine,point);
        plfh.distance_histogram.push_back(dis_line);
    }
    return plfh;
}


void Calcu_PLFHset(plane_set& PlaneSet, vector<pcl::PLFH_gather>& plfh_connected_set, pcl::PointXYZ point)
{
    pcl::PLFH_scattered plfh;
    pcl::PLFH_gather plfh_connected;
   
    for (int i = 0; i < PlaneSet.getPlaneNumber(); i++) {
        plfh=Calcu_plfh( i,PlaneSet,point);//计算角度和距离
        plfh_connected.copyToFloatArray(plfh); // 连接直方图
        plfh_connected_set.push_back(plfh_connected);
        plfh.angle_histogram.clear();
        plfh.distance_histogram.clear();
        plfh_connected.features_histogram.clear();
        plfh_connected.nr_dimensions_ = 0;
    }
}
