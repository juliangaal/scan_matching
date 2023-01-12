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

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
bool next_iteration = false;

template <typename T>
void preprocess(typename pcl::PointCloud<T>::Ptr &cloud, float scale = 1000.f)
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
}

// void
// keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
//                        void* nothing)
// {
//   //if (event.getKeySym () == "space" && event.keyDown ())
  
//   next_iteration = true;
// }

int main()
{
    float grid_size = 1;
    int iterations = 20;
    float scale = 1000.f;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
    file2pcd<pcl::PointXYZ>(model, "../data/bun000_UnStructured.pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    file2pcd<pcl::PointXYZ>(scene, "../data/bun045_UnStructured.pcd");
    
    preprocess<pcl::PointXYZ>(model, scale);
    preprocess<pcl::PointXYZ>(scene, scale);
    
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

    ICP_LM icp(model, 1, iterations);
    icp.align(scene);    
    return 0;
}
