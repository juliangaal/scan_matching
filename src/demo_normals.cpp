#include "utils.h"
#include "viewer.h"

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>

int main(void)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    file2pcd<pcl::PointXYZ>(cloud, "/home/julian/dev/thesis_playground/registration/registration/data/carpark_cloud_velodyne_hdl_32e.pcd");

    Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::demeanPointCloud(*cloud, centroid, *cloud);

    // calculate normals
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // tree is empty now but will be filled after normal calculation
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(50);
    ne.setViewPoint(centroid.x(), centroid.y(), centroid.z());
    ne.compute(*normals);

    Viewer viewer("PCL Viewer");
    viewer.add_pointcloud("cloud", cloud, 1.0, 255, 0, 0);
    viewer.add_normals<pcl::PointXYZ>("cloud normals", cloud, normals, 1, 0.03);

    viewer.show_viewer();
}

