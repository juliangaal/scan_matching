//
// Created by julian on 29.03.22.
//

#include "icp_LM.h"
#include "utils.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>

ICP_LM::ICP_LM(int grid_size, int distance_threshold, int iterations)
: _grid_size(grid_size)
, _distance_threshold(distance_threshold)
, _iterations(iterations)
, _state(Eigen::Matrix<float, 6, 1>::Zero())
{
}

void ICP_LM::align(pcl::PointCloud<pcl::PointXYZ>::Ptr &model, pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                   Eigen::Vector4f &centroid)
{
    // voxel filtering (model)
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(model);
    sor.setLeafSize (_grid_size, _grid_size, _grid_size);
    sor.filter(*model);

    // voxel filter (
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
    ne.setRadiusSearch(0.03);
    ne.setViewPoint(centroid.x(), centroid.y(), centroid.z());
    ne.compute(*normals);

    // for every point in scene, get closest point in model
    std::vector<int> indices(scene->size());
    std::iota(indices.begin(), indices.end(), 0);
    std::vector<std::vector<int>> nns;
    std::vector<std::vector<int>> point_idx_search(scene->size());
    std::vector<std::vector<float>> point_squared_distance(scene->size());
    tree->nearestKSearch(*scene, indices, 1, point_idx_search, point_squared_distance);

    auto H = Eigen::Matrix<float, 6, 6>::Zero();
    auto b = Eigen::Matrix<float, 6, 1>::Zero();

    // correspondences (nearest neighbor, normals) are calculated
    for (int i = 0; i < point_idx_search.size(); ++i)
    {
        auto& curr_scene_point = (*scene)[i];
        auto& corr_point = (*model)[point_idx_search[i][0]];
        auto& corr_normal = (*normals)[point_idx_search[i][0]];
    }

    auto [R, t] = state_2_rot_trans(_state);


    rot_trans_2_state(_state, R, t);
}

