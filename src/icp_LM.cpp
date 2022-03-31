//
// Created by julian on 29.03.22.
//

#include "icp_LM.h"
#include "utils.h"

#include <fmt/printf.h>
#include <fmt/ostream.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>

ICP_LM::ICP_LM(int grid_size, int distance_threshold, int iterations)
: _grid_size(grid_size)
, _distance_threshold(distance_threshold)
, _iterations(iterations)
, _state(State::Zero())
{
}

void ICP_LM::align(pcl::PointCloud<pcl::PointXYZ>::Ptr &model, pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                   Eigen::Vector4f &centroid)
{
    fmt::print("test\n");
    // voxel filtering (model)
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(model);
    sor.setLeafSize (_grid_size, _grid_size, _grid_size);
    sor.filter(*model);

    // voxel filter (scene)
    sor.setInputCloud(scene);
    sor.setLeafSize (_grid_size, _grid_size, _grid_size);
    sor.filter(*scene);

    // calculate normals
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // tree is empty now but will be filled after normal calculation
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setInputCloud(model);
    ne.setSearchMethod(tree);
    ne.setKSearch(10);
    ne.setViewPoint(centroid.x(), centroid.y(), centroid.z());
    ne.compute(*normals);

    float lambda = 1e-2;

    for (int j = 0; j < _iterations; j++)
    {
        // for every point in scene, get closest point in model
        std::vector<int> indices(scene->size());
        std::iota(indices.begin(), indices.end(), 0);
        std::vector<std::vector<int>> nns;
        std::vector<std::vector<int>> point_idx_search(scene->size());
        std::vector<std::vector<float>> point_squared_distance(scene->size());
        tree->nearestKSearch(*scene, indices, 1, point_idx_search, point_squared_distance);

        Hessian H = Hessian::Zero();
        State gradient = State::Zero();

        auto [ R, t ] = state_2_rot_trans(_state);
        float chi = 0.f;

        // correspondences (nearest neighbor, normals) are calculated
        for (int i = 0; i < point_idx_search.size(); ++i)
        {
            const Eigen::Vector3f &scene_point = ((*scene)[i]).getVector3fMap();
            const Eigen::Vector3f &corr_point = ((*model)[point_idx_search[i][0]]).getVector3fMap();
            const Eigen::Vector3f &corr_normal = ((*normals)[point_idx_search[i][0]]).getNormalVector3fMap();

            // rotate scene point with current best guess
            auto rot_scene_point = R * scene_point;
            float error = (rot_scene_point + t - corr_point).dot(corr_normal);

            Eigen::Vector6f J = jacobian_plane(rot_scene_point, corr_normal);
            H += J.transpose() * J;
            gradient += J.transpose() * error;
            chi += std::abs(error);
        }

        H += lambda * H * Hessian::Identity();
        State update = -H.inverse() * gradient;
        State state_new = _state + update;
        auto [ R_new, t_new ] = state_2_rot_trans(state_new);

        float chi_new = 0;

        for (int i = 0; i < point_idx_search.size(); ++i)
        {
            const Eigen::Vector3f &scene_point = ((*scene)[i]).getVector3fMap();
            const Eigen::Vector3f &corr_point = ((*model)[point_idx_search[i][0]]).getVector3fMap();
            const Eigen::Vector3f &corr_normal = ((*normals)[point_idx_search[i][0]]).getNormalVector3fMap();

            auto rot_scene_point_new = R_new * scene_point;
            float error_new = (rot_scene_point_new + t_new - corr_point).dot(corr_normal);
            chi_new += std::abs(error_new);
        }

        if (chi_new > chi)
        {
            lambda *= 10;
        }
        else
        {
            _state = state_new;
            lambda /= 10;
        }

        fmt::print("Error: {}, lambda: {}\n", chi / (float)point_idx_search.size(), lambda);

        std::tie(R, t) = state_2_rot_trans(_state);

        Eigen::Matrix4f pcl_transform = Eigen::Matrix4f::Identity();
        pcl_transform.block<3, 3>(0, 0) = R;
        pcl_transform.block<3, 1>(0, 3) = t;

        // update pcl for next iteration
        pcl::transformPointCloud(*scene, *scene, pcl_transform);
    }
}

