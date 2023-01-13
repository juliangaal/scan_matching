//
// Created by julian on 29.03.22.
//

#include "viewer.h"
#include "icp_lm.h"
#include "icp_lm_params.h"
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
#include <pcl/console/time.h>  

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

    // demean pcl for normal calculation
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

int main()
{
    ICP_LM_Params params(std::filesystem::path(PARAMS_DIR) / "icp_lm.toml");
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
    file2pcd<pcl::PointXYZ>(model, "../data/bun000_UnStructured.pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    file2pcd<pcl::PointXYZ>(scene, "../data/bun045_UnStructured.pcd");
    
    preprocess<pcl::PointXYZ>(model, params.scale);
    preprocess<pcl::PointXYZ>(scene, params.scale);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_t(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*model, *model_t);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_t(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*scene, *scene_t);
    
    // voxel filtering
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(model_t);

    sor.setLeafSize (params.voxel_size, params.voxel_size, params.voxel_size);
    sor.filter(*model_t);

    sor.setInputCloud(scene_t);
    sor.filter(*scene_t);

    ICP_LM icp(model, params.distance_threshold, params.iterations);
    icp.align(scene);    
    return 0;
}
