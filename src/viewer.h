#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

/**
  * @file viewer.h
  * @author julian 
  * @date 1/31/22
 */

class ScanMatchingViewer
{
public:
    explicit ScanMatchingViewer(const std::string &name);
    
    ~ScanMatchingViewer() = default;
    
    void add_model(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double size = 1.0);

    void add_scene(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double size = 1.0);

    void
    add_pointcloud(std::string id, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double size, int r,
                           int g,
                           int b);

    void update(int iterations, float error, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Eigen::Matrix4f& transform);

    void set_point_size(std::string id, double size);

    template <typename T>
    inline void add_normals(std::string id, const typename pcl::PointCloud<T>::Ptr &cloud,
                const pcl::PointCloud<pcl::Normal>::Ptr &normals, int level, float scale)
    {
        _viewer.addPointCloudNormals<T, pcl::Normal>(cloud, normals, level, scale, id);
    }

    pcl::visualization::PCLVisualizer& viewer();
    
private:
    pcl::visualization::PCLVisualizer _viewer;
    static constexpr float _bckgr_gray_level = 0.0;
    static constexpr float _txt_gray_lvl = 1.0 - _bckgr_gray_level;
};
