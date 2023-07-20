#pragma once
#include <cmath>
#include <vector>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include "plane_set.h"

typedef struct point_ {
    double x;
    double y;
    double z;
}point_line;

typedef struct linenormal {
    double normalX;
    double normalY;
    double normalZ;
}LineNormal;


typedef struct linedirection {
    double dierctionlX;
    double dierctionlY;
    double dierctionlZ;
}LineDirection;


class LinePara3D {
public:
    vector<point_line> point_several;
    LineNormal line_normal;
    LineDirection line_direction;

    //��������Ϊ(a,b,c)����������ȡΪn = (b, -a, 0) �� n = (c, 0, -a)
    LinePara3D() {
        point_several.clear();
    }
};


namespace pcl
{
    class PLFH_scattered
    {
    public:
        std::vector<float> angle_histogram;     // �Ƕ�ֱ��ͼ
        std::vector<float> distance_histogram;  // ����ֱ��ͼ

        PLFH_scattered()
        {
            // ��ʼ��ֱ��ͼ�Ĵ�СΪ10��bin
            angle_histogram.resize(10, 0.0f);
            distance_histogram.resize(10, 0.0f);
        }
    };

    template <>
    class DefaultPointRepresentation<PLFH_scattered> : public PointRepresentation<PLFH_scattered>
    {
        using PointRepresentation<PLFH_scattered>::nr_dimensions_;

    public:
        //vector<float> plfh_connected;
        DefaultPointRepresentation()
        {
            // ÿ����������������ֱ��ͼ������ά��Ϊ20
            nr_dimensions_ = 20;
            //plfh_connected.clear();
        }

        int get_nr_dimensions_() {
            return nr_dimensions_;
        };

        // ������������ת��Ϊ��������
        void copyToFloatArray(const PLFH_scattered& f, float* out) const
        {
            for (size_t i = 0; i < f.angle_histogram.size(); ++i)
            {
                out[i] = f.angle_histogram[i];
                //plfh_connected.push_back(out[i]);
            }

            for (size_t i = 0; i < f.distance_histogram.size(); ++i)
            {
                out[f.angle_histogram.size() + i] = f.distance_histogram[i];
            }
        }
    };
} 

void Calcu_PLFHset(plane_set& PlaneSet, vector<vector<std::float_t>>& plfh_connected_set, pcl::PointXYZ point) {
    /*pcl::PLFH_scattered plfh;
    pcl::DefaultPointRepresentation<pcl::PLFH_scattered> plfh_connected;
    vector<float> plfh_out;

    for (int i = 0; i < PlaneSet.getPlaneNumber(); i++) {
        plfh = Calcu_plfh(PlaneSet.getPlaneSet()[i], i, PlaneSet, point);//����ǶȺ;���
        plfh_connected.copyToFloatArray(plfh, plfh_out.data()); // ����ֱ��ͼ
        plfh_connected_set.push_back(plfh_out);
    }*/
}