//
// Created by julian on 29.03.22.
//

#include "viewer.h"
#include "icp_LM.h"
#include "utils.h"

#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

template <typename T>
Eigen::Vector4f preprocess(typename pcl::PointCloud<T>::Ptr &cloud, float scale = 1000.f)
{
    // remove NaNs
    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // demean pcl
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::demeanPointCloud(*cloud, centroid, *cloud);

    // scale
    for (auto& p: cloud->points)
    {
        p.x *= scale;
        p.y *= scale;
        p.z *= scale;
    }

    return centroid;
}

int main()
{
    float grid_size = 1;
    int iterations = 20;
    float scale = 1000.f;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
    file2pcd<pcl::PointXYZ>(model, "../data/bun000_UnStructured.pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());;
    file2pcd<pcl::PointXYZ>(scene, "../data/bun045_UnStructured.pcd");
    
    auto centroid = preprocess<pcl::PointXYZ>(model, scale);
    preprocess<pcl::PointXYZ>(scene, scale);
    
    /**
     * For visualization only
     * - copy original model and scene cloud
     * - transform to side for viewing side by side in viewer
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_model(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*model, *origin_model);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_scene(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*scene, *origin_scene);
    
    // voxel filtering
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(origin_model);
    sor.setLeafSize (grid_size, grid_size, grid_size);
    sor.filter(*origin_model);
    sor.setInputCloud(origin_scene);
    sor.filter(*origin_scene);

    /**
     * Run ICP
     * args: grid_size, dist_thresh, iterations, model, scene, viewer
    */
    ICP_LM icp(grid_size, 1, iterations);
    Eigen::Matrix4f transform = icp.align(model, scene, centroid);
    pcl::transformPointCloud(*origin_scene, *origin_scene, transform);
    
    // transform out of the way (for visualization only)
    Eigen::Matrix4f viz_transform = Eigen::Matrix4f::Identity();
    viz_transform(0, 3) += .2f * scale;
    pcl::transformPointCloud(*model, *model, viz_transform);
    pcl::transformPointCloud(*scene, *scene, viz_transform);
    
    Viewer viewer("PCL Viewer");
//    viewer.add_pointcloud("model cloud", model, 1.0, 0, 0, 255);
//    viewer.add_pointcloud("scene cloud", scene, 1.0, 255, 0, 0);
    viewer.add_pointcloud("model cloud original", origin_model, 1.0, 0, 0, 255);
    viewer.add_pointcloud("scene cloud original", origin_scene, 1.0, 255, 0, 0);
    viewer.show_viewer();
    
    return 0;
}
