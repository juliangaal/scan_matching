//
// Created by julian on 29.03.22.
//

#include "viewer.h"
#include "icp_LM.h"

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <fmt/printf.h>

template <typename T>
void preprocess(typename pcl::PointCloud<T>::Ptr &cloud, float scale = 1000.f)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::demeanPointCloud(*cloud, centroid, *cloud);
    std::for_each(cloud->points.begin(), cloud->points.end(), [&](auto& p)
    {
        p.x *= scale; p.y *= scale; p.z *= scale;
    });
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

    preprocess<pcl::PointXYZ>(model);
    preprocess<pcl::PointXYZ>(scene);

    // args: grid_size, dist_thresh, iterations, model, scene, viewer

    Viewer viewer("PCL Viewer");
    viewer.add_pointcloud("model cloud", model, 3.0, 0, 0, 255);
    viewer.add_pointcloud("scene cloud", scene, 4.0, 255, 0, 0);
    viewer.show_viewer();

    return 0;
}
