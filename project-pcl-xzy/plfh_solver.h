#pragma once
#include <cmath>
#include <vector>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include "plane_set.h"

namespace pcl
{
    class PLFH_scattered
    {
    public:
        std::vector<float> angle_histogram;     // 角度直方图
        std::vector<float> distance_histogram;  // 距离直方图

        PLFH_scattered()
        {
            // 初始化直方图的大小为10个bin
            angle_histogram.resize(10, 0.0f);
            distance_histogram.resize(10, 0.0f);
        }
    };


    class DefaultPointRepresentation<PLFH_scattered> : public PointRepresentation<PLFH_scattered>
    {
        using PointRepresentation<PLFH_scattered>::nr_dimensions_;

    public:
        DefaultPointRepresentation()
        {
            // 每个描述符包含两个直方图，所以维度为20
            nr_dimensions_ = 20;
        }

        // 将特征描述符转换为特征向量
        void copyToFloatArray(const PLFH_scattered& f, float* out) const
        {
            for (size_t i = 0; i < f.angle_histogram.size(); ++i)
            {
                out[i] = f.angle_histogram[i];
            }

            for (size_t i = 0; i < f.distance_histogram.size(); ++i)
            {
                out[f.angle_histogram.size() + i] = f.distance_histogram[i];
            }
        }
    };
} 