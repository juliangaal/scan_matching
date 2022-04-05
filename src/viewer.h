#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

/**
  * @file viewer.h
  * @author julian 
  * @date 1/31/22
 */

class Viewer
{
public:
    explicit Viewer(const std::string &name);
    
    ~Viewer() = default;
    
    void add_pointcloud(std::string id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, double size);

    void
    add_pointcloud(std::string id, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double size, int r,
                           int g,
                           int b);

    void set_point_size(std::string id, double size);

    template <typename T>
    inline void add_normals(std::string id, const typename pcl::PointCloud<T>::Ptr &cloud,
                const pcl::PointCloud<pcl::Normal>::Ptr &normals, int level, float scale)
    {
        viewer.addPointCloudNormals<T, pcl::Normal>(cloud, normals, level, scale, id);
    }
    
    void show_viewer();
    
    void add_point(const std::string &id, Eigen::Vector4d &matrix, double size, int r, int g, int b);

private:
    pcl::visualization::PCLVisualizer viewer;
};
