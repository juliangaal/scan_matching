#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

struct ICP_LM
{
    ICP_LM(float grid_size, int distance_threshold, int iterations);
    ~ICP_LM() = default;

    Eigen::Matrix4f align(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scene);

    int _distance_threshold;
    int _iterations;
    Eigen::Matrix<float, 6, 1> _state;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _model;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _scene;
    pcl::VoxelGrid<pcl::PointXYZ> _voxel_filter;
    
    void set_model(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &model) const;
};
