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
}

int main(void)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
    file2pcd<pcl::PointXYZ>(model, "../data/bun000_UnStructured.pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());;
    file2pcd<pcl::PointXYZ>(scene, "../data/bun045_UnStructured.pcd");

    auto centroid = preprocess<pcl::PointXYZ>(model);
    preprocess<pcl::PointXYZ>(scene);

    // args: grid_size, dist_thresh, iterations, model, scene, viewer
    ICP_LM icp(4, 1, 20);
    icp.align(model, scene, centroid);

//    Viewer viewer("PCL Viewer");
//    viewer.add_pointcloud("model cloud", model, 3.0, 0, 0, 255);
//    viewer.add_pointcloud("scene cloud", scene, 4.0, 255, 0, 0);
//    viewer.show_viewer();


    return 0;
}
