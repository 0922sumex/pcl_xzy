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
        std::vector<float> angle_histogram;     // �Ƕ�ֱ��ͼ
        std::vector<float> distance_histogram;  // ����ֱ��ͼ

        PLFH_scattered()
        {
            // ��ʼ��ֱ��ͼ�Ĵ�СΪ10��bin
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
            // ÿ����������������ֱ��ͼ������ά��Ϊ20
            nr_dimensions_ = 20;
        }

        // ������������ת��Ϊ��������
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