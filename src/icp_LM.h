#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct ICP_LM
{
    ICP_LM(float grid_size, int distance_threshold, int iterations);
    ~ICP_LM() = default;

    Eigen::Matrix4f align(pcl::PointCloud<pcl::PointXYZ>::Ptr &model, pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                          Eigen::Vector4f &centroid);

    float _grid_size;
    int _distance_threshold;
    int _iterations;
    Eigen::Matrix<float, 6, 1> _state;
};
