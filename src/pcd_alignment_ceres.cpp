//
// Created by julian on 29.03.22.
//

#include "viewer.h"
#include "icp_LM.h"
#include "utils.h"

#include <thread>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

using namespace std::chrono_literals;

template <typename T>
void preprocess(typename pcl::PointCloud<T>::Ptr &cloud, float scale = 1000.f)
{
    // scale
    for (auto& p: cloud->points)
    {
        p.x *= scale;
        p.y *= scale;
        p.z *= scale;
    }
}

int main()
{
    float grid_size = 0.1;
    int iterations = 5;
    float distance_threshold = 1;
    float scale = 1.f;

    pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
    file2pcd<pcl::PointXYZ>(model, "/home/julian/documents/frame_0.pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());;
    file2pcd<pcl::PointXYZ>(scene, "/home/julian/documents/frame_40.pcd");

    preprocess<pcl::PointXYZ>(model, scale);
    preprocess<pcl::PointXYZ>(scene, scale);

    /**
     * For visualization only
     * - copy original model and scene cloud
     * - transform to side for viewing side by side in viewer
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_t(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*model, *model_t);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_t(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*scene, *scene_t);

    // voxel filtering
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(model_t);
    sor.setLeafSize (grid_size, grid_size, grid_size);
    sor.filter(*model_t);
    sor.setInputCloud(scene_t);
    sor.filter(*scene_t);

    /**
     * Run ICP
     * args: grid_size, dist_thresh, iterations, model, scene, viewer
    */
    ICP_LM icp(grid_size, distance_threshold, iterations);
    icp.set_model(model);
    Eigen::Matrix4f transform = icp.align_ceres(scene);
    pcl::transformPointCloud(*scene_t, *scene_t, transform);


    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 3.0, 1.0, v2);
    viewer.setCameraPosition(-400.539, 381.124, 460.384, 0.285877, 0.868995, -0.403883, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_color(model, 0, 0, 255);
    viewer.addPointCloud(model, model_color, "model_color", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_color(scene, 255, 0, 0);
    viewer.addPointCloud(scene, scene_color, "scene_color", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_color_t(model_t, 0, 0, 255);
    viewer.addPointCloud(model_t, model_color_t, "model_color_t", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_color_t(scene_t, 255, 0, 0);
    viewer.addPointCloud(scene_t, scene_color_t, "scene_color_t", v2);
    viewer.addCoordinateSystem();

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}
