
/**
  * @file viewer.cpp
  * @author julian 
  * @date 1/31/22
 */

#include "viewer.h"
#include <thread>

Viewer::Viewer(const std::string &name)
    : viewer(name)
{
    viewer.setBackgroundColor(0, 0, 0);
}

void Viewer::add_pointcloud(std::string id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, double size)
{
    pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, id);
    set_point_size(id, size);
}

void
Viewer::add_pointcloud(std::string id, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double size, int r,
                               int g,
                               int b)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, r, g, b);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, single_color, id);
    set_point_size(id, size);
}

void Viewer::set_point_size(std::string id, double size)
{
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id);
}

void Viewer::add_normals(std::string id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                         const pcl::PointCloud<pcl::Normal>::Ptr &normals, int level, float scale)
{
    viewer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud, normals, level, scale, id);
}

void Viewer::show_viewer()
{
    viewer.addCoordinateSystem();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Viewer::add_point(const std::string &id, Eigen::Vector4d &matrix, double size, int r, int g, int b)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointXYZRGBA ptemp;
    ptemp.x = matrix.x();
    ptemp.y = matrix.y();
    ptemp.z = matrix.z();
    ptemp.r = r;
    ptemp.g = g;
    ptemp.b = b;
    ptemp.a = 255;
    temp->points.push_back(ptemp);
    add_pointcloud(id, temp, size);
}
