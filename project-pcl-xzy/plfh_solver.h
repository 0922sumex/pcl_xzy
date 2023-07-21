#pragma once
#include <cmath>
#include <vector>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <algorithm>
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

    //方向向量为(a,b,c)，法向量可取为n = (b, -a, 0) 或 n = (c, 0, -a)
    LinePara3D() {
        point_several.clear();
        line_normal.normalX = 0;
        line_normal.normalY = 0;
        line_normal.normalZ = 0;
        line_direction.dierctionlX = 0;
        line_direction.dierctionlY = 0;
        line_direction.dierctionlZ = 0;
    }
};


namespace pcl
{
    class PLFH_scattered
    {
    public:
        std::vector<float> angle_histogram;     // 角度直方图
        std::vector<float> distance_histogram;  // 距离直方图
        int nr_dimensions_;

        PLFH_scattered()
        {
            // 初始化直方图的大小为10个bin
            nr_dimensions_ = 0;
            angle_histogram.clear();
            distance_histogram.clear();
        }
    };

    class PLFH_gather
    {
    public:
        std::vector<float> features_histogram;     // 特征直方图
        int nr_dimensions_;

        PLFH_gather()
        {
            // 初始化直方图的大小为10个bin
            features_histogram.clear();
            nr_dimensions_ = 0;
        }

        // 将特征描述符转换为特征向量
        void copyToFloatArray(const PLFH_scattered& f)
        {
            for (size_t i = 0; i < f.angle_histogram.size(); ++i)
            {
                features_histogram.push_back(f.angle_histogram[i]);
                nr_dimensions_++;
            }

            for (size_t i = 0; i < f.distance_histogram.size(); ++i)
            {
                features_histogram.push_back(f.distance_histogram[i]);
                nr_dimensions_++;
            }
        }
    };
}
   
void Calcu_PLFHset(plane_set& PlaneSet, vector<pcl::PLFH_gather>& plfh_connected_set, pcl::PointXYZ point);