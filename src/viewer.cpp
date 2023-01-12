
/**
  * @file _viewer.cpp
  * @author julian 
  * @date 1/31/22
 */

#include "viewer.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/common/transforms.h>
#include <string>
#include <thread>

ScanMatchingViewer::ScanMatchingViewer(const std::string &name)
    : _viewer(name)
{
    _viewer.setBackgroundColor(0, 0, 0);
    _viewer.addText ("Blue: Original point cloud\nRed: transformed cloud", \
    10, 15, 16, _txt_gray_lvl, _txt_gray_lvl, _txt_gray_lvl, "icp_info_1");
    _viewer.addText("Iterations = 0", 10, 65, 16, _txt_gray_lvl, _txt_gray_lvl, _txt_gray_lvl, "iterations_cnt");
    _viewer.addText("Error      = 0", 10, 50, 16, _txt_gray_lvl, _txt_gray_lvl, _txt_gray_lvl, "error");
}

void ScanMatchingViewer::add_model(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double size)
{
    add_pointcloud("model", cloud, size, 0, 0, 255);
    set_point_size("model", size);
}

void ScanMatchingViewer::add_scene(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double size)
{
    add_pointcloud("scene", cloud, size, 255, 0, 0);
    set_point_size("scene", size);
}

void ScanMatchingViewer::update(int iterations, float error, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Eigen::Matrix4f& transform)
{
    std::string iterations_cnt = "Iterations = " + std::to_string(iterations+1);
    std::string iteration_error = "Error = " + std::to_string(error);
    _viewer.updateText(iterations_cnt, 10, 65, 16, _txt_gray_lvl, _txt_gray_lvl, _txt_gray_lvl, "iterations_cnt");
    _viewer.updateText(iteration_error, 10, 50, 16, _txt_gray_lvl, _txt_gray_lvl, _txt_gray_lvl, "error");

    pcl::transformPointCloud(*cloud, *cloud, transform);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
    _viewer.updatePointCloud(cloud, single_color, "scene");
}

pcl::visualization::PCLVisualizer& ScanMatchingViewer::viewer()
{
    return _viewer;
}

void
ScanMatchingViewer::add_pointcloud(std::string id, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double size, int r,
                               int g,
                               int b)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, r, g, b);
    _viewer.addPointCloud<pcl::PointXYZ>(cloud, single_color, id);
    set_point_size(id, size);
}

void ScanMatchingViewer::set_point_size(std::string id, double size)
{
    _viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id);
}
