//
// Created by julian on 29.03.22.
//

#include "viewer.h"
#include "icp_LM.h"

#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <fmt/printf.h>
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

template <typename T>
void file2pcd(typename pcl::PointCloud<T>::Ptr& cloud, std::string filename)
{
    if (pcl::io::loadPCDFile<T>(filename, *cloud) == -1)
    {
        fmt::print("Couldn't read file {}\n", filename);
        throw std::runtime_error("PCL IO Error");
    }
    
    fmt::print("Loaded {} with {} points\n", filename, cloud->size());
}

int main()
{
    float grid_size = 4;
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
    
    // transform
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) -= .2f * scale;
    pcl::transformPointCloud(*origin_model, *origin_model, transform);
    pcl::transformPointCloud(*origin_scene, *origin_scene, transform);

    /**
     * Run ICP
     * args: grid_size, dist_thresh, iterations, model, scene, viewer
    */
    ICP_LM icp(grid_size, 1, iterations);
    icp.align(model, scene, centroid);

    Viewer viewer("PCL Viewer");
    viewer.add_pointcloud("model cloud", model, 3.0, 0, 0, 255);
    viewer.add_pointcloud("scene cloud", scene, 3.0, 255, 0, 0);
    viewer.add_pointcloud("model cloud original", origin_model, 3.0, 0, 0, 255);
    viewer.add_pointcloud("scene cloud original", origin_scene, 3.0, 255, 0, 0);
    viewer.show_viewer();
    
    return 0;
}
