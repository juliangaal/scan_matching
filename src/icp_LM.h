#pragma once

#include "viewer.h"
#include <ceres/ceres.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

struct Point2PlaneFactor
{

    Point2PlaneFactor(Eigen::Vector3d scene_point, Eigen::Vector3d model_point,
                      Eigen::Vector3d model_normal)
    : scene_point(scene_point)
    , model_point(model_point)
    , model_normal(model_normal) {}

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr(t[0], t[1], t[2]);
        Eigen::Matrix<T, 3, 1> sp(T(scene_point.x()), T(scene_point.y()), T(scene_point.z()));
        Eigen::Matrix<T, 3, 1> mp(T(model_point.x()), T(model_point.y()), T(model_point.z()));
        Eigen::Matrix<T, 3, 1> n(T(model_normal.x()), T(model_normal.y()), T(model_normal.z()));

        residual[0] = (q_w_curr * sp + t_w_curr - mp).dot(n);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d scene_point, const Eigen::Vector3d model_point,
                                       const Eigen::Vector3d model_normal)
    {
        return (new ceres::AutoDiffCostFunction<
                Point2PlaneFactor, 1, 4, 3>(
                new Point2PlaneFactor(scene_point, model_point, model_normal)));
    }

    Eigen::Vector3d scene_point;
    Eigen::Vector3d model_point;
    Eigen::Vector3d model_normal;
};

struct ICP_LM
{
    ICP_LM(const pcl::PointCloud<pcl::PointXYZ>::Ptr &model, float distance_threshold, int iterations);
    ~ICP_LM() = default;

    Eigen::Matrix4f align(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene);
    Eigen::Matrix4f align_ceres(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene);

    float _distance_threshold;
    int _iterations;
    Eigen::Matrix<float, 6, 1> _state;
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& _model;
    pcl::VoxelGrid<pcl::PointXYZ> _voxel_filter;
    ScanMatchingViewer _viewer;
};
