#include "plfh_solver.h"


void calcLine(pcl::ModelCoefficients coefsOfPlane1, pcl::ModelCoefficients coefsOfPlane2, LinePara3D& coefsOfLine)
{
    //�����ƽ���Ľ���
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
    //ֱ����һ��
    point_line point_temp;
    point_temp.x = 0.0;
    point_temp.y = tempy;
    point_temp.z = tempz;
    coefsOfLine.point_several.push_back(point_temp);//���ߵ�һ���㣨0,y,z��
    //���ߵķ�������
    coefsOfLine.line_direction.dierctionlX=b1 * c2 - c1 * b2;
    coefsOfLine.line_direction.dierctionlY=c1 * a2 - a1 * c2;
    coefsOfLine.line_direction.dierctionlZ=a1 * b2 - b1 * a2;

    //���ߵķ�����  (b, -a, 0) 
    coefsOfLine.line_normal.normalX= c1 * a2 - a1 * c2;
    coefsOfLine.line_normal.normalY = c1 * b2 - b1 * c2;
    coefsOfLine.line_normal.normalZ = 0;
}


double Point2Line3DVecproduct(LinePara3D line, pcl::PointXYZ point)
{
    //����̶��㵽���ߵľ���
    //ֱ�ߵķ�����(p,q,r)
    double p = line.line_normal.normalX;
    double q = line.line_normal.normalY;
    double r = line.line_normal.normalZ;

    //ֱ���ϵ�2����q��b
    double x_q = line.point_several[0].x;
    double y_q = line.point_several[0].y;
    double z_q = line.point_several[0].z;

    double x_b = 1; //���2�����x=1
    double y_b = y_q - x_q * q / p + q / p;
    double z_b = z_q - x_q * r / p + r / p;

    double x_o = point.x;
    double y_o = point.y;
    double z_o = point.z;

    //��������
    //oq
    double normal01_x = x_q - x_o;
    double normal01_y = y_q - y_o;
    double normal01_z = z_q - z_o;

    //ob
    double normal02_x = x_b - x_o;
    double normal02_y = y_b - y_o;
    double normal02_z = z_b - z_o;

    //oq��ob���в��
    double chacheng_x = normal01_y * normal02_z - normal02_y * normal01_z;
    double chacheng_y = normal02_z * normal01_x - normal02_x * normal01_z;
    double chacheng_z = normal01_x * normal02_y - normal01_y * normal02_x;

    double chacheng_length = sqrt(chacheng_x * chacheng_x + chacheng_y * chacheng_y + chacheng_z * chacheng_z);
    double dx = x_q - x_b;
    double dy = y_q - y_b;
    double dz = z_q - z_b;

    double qb_length = sqrt(dx * dx + dy * dy + dz * dz);

    double ds = chacheng_length / qb_length;

    //cout << "����̶��㵽ƽ��佻�ߵľ��룺" << ds << endl;

    return ds;

}

pcl::PLFH_scattered Calcu_plfh (int index,plane_set PlaneSet, pcl::PointXYZ point) {

    //�����ƽ�浽����ƽ��֮��ĽǶȺ;���

    pcl::PLFH_scattered plfh;
    plane Plane_itself=PlaneSet.getPlaneSet()[index];
    Eigen::Vector3f n1 = Plane_itself.normal;
    LinePara3D coefsOfLine;//�ཻ�� 

    for (int i = 0; i < PlaneSet.getPlaneNumber(); i++) {
        if (i == index) {
            continue;//������������
        }
        Eigen::Vector3f n2 = PlaneSet.getPlaneSet()[i].normal;
        // ��һ������
        n1.normalize();
        n2.normalize();
        float dotProduct = n1.dot(n2);
        float n1Norm = n1.norm();
        float n2Norm = n2.norm();

        // ʹ�÷����Һ�������н�
        float angle = std::acos(dotProduct / (n1Norm * n2Norm));
        plfh.angle_histogram.push_back(angle);
        plfh.nr_dimensions_++;

        // ����һ���̶��㵽ƽ���ཻ��֮��ľ���
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
        plfh=Calcu_plfh( i,PlaneSet,point);//����ǶȺ;���
        plfh_connected.copyToFloatArray(plfh); // ����ֱ��ͼ
        plfh_connected_set.push_back(plfh_connected);
        plfh.angle_histogram.clear();
        plfh.distance_histogram.clear();
        plfh_connected.features_histogram.clear();
        plfh_connected.nr_dimensions_ = 0;
    }
}
