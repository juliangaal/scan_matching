
// local lib
#include "sac_ia_gicp_params.h"
#include <filesystem>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/gicp.h>

using Feature = pcl::FPFHSignature33;
using pointcloud = pcl::PointCloud<pcl::PointXYZ>;
using pointnormal = pcl::PointCloud<pcl::Normal>;
using fpfhFeature = pcl::PointCloud<Feature>;

fpfhFeature::Ptr compute_fpfh_feature(const pointcloud::Ptr& input_cloud, const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree)
{
    pointnormal::Ptr normals(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(input_cloud);
    n.setNumberOfThreads(8);
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals);

    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    f.setNumberOfThreads(8);
    f.setInputCloud(input_cloud);
    f.setInputNormals(normals);
    f.setSearchMethod(tree);
    f.setKSearch(10);
    f.compute(*fpfh);
    return fpfh;
}

int main(void)
{   
    SAC_IA_GICP_Params params(std::filesystem::path(PARAMS_DIR) / "sac_ia_gicp.toml");

    clock_t start_sac_ia, end_sac_ia,start_icp, end_icp;

    pointcloud::Ptr source_cloud(new pointcloud);
    pointcloud::Ptr target_cloud(new pointcloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/julian/dev/warpsense_ws/src/fast_gicp/data/251370668.pcd", *source_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/julian/dev/warpsense_ws/src/fast_gicp/data/251371071.pcd", *target_cloud);

    pcl::Indices indices_src; 
    pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
    cout << "remove *source_cloud nan" << endl;
    pcl::Indices indices_tgt;
    pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
    cout << "remove *target_cloud nan" << endl;

    pcl::VoxelGrid<pcl::PointXYZ> vg_source;
    vg_source.setLeafSize(10.0, 10.0, 10.0);
    vg_source.setInputCloud(source_cloud);
    pointcloud::Ptr vg_source_cloud(new pointcloud);
    vg_source.filter(*vg_source_cloud);
    cout << "down size *source_cloud from " << source_cloud->size() << " to " <<vg_source_cloud->size() << endl;

    pcl::VoxelGrid<pcl::PointXYZ> vg_target;
    vg_target.setLeafSize(10.0, 10.0, 10.0);
    vg_target.setInputCloud(target_cloud);
    pointcloud::Ptr vg_target_cloud(new pointcloud);
    vg_target.filter(*vg_target_cloud);
    cout << "down size *target_cloud from " << target_cloud->size() << " to " << vg_target_cloud->size() << endl;
         
    Eigen::Matrix4f t;
    t << 1, 0, 0, 25,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    
    pcl::transformPointCloud(*vg_target_cloud, *vg_target_cloud, t);
   

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(vg_source_cloud, tree);
    fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(vg_target_cloud, tree);
    cout<<"fpfh has finished"<<endl;

    start_sac_ia = clock();
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(vg_source_cloud);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(vg_target_cloud);
    sac_ia.setTargetFeatures(target_fpfh);
    sac_ia.setMinSampleDistance(0.1);
    //sac_ia.setNumberOfSamples(200); 
    sac_ia.setCorrespondenceRandomness(6);
    pointcloud::Ptr sac_ia_cloud(new pointcloud);
    sac_ia.align(*sac_ia_cloud);

    end_sac_ia = clock();
    std::cerr << "calculate time is: " << float(end_sac_ia - start_sac_ia) / CLOCKS_PER_SEC <<"s"<< endl;
    std::cerr << std::endl << "SAC-IA has converged, score : " << sac_ia.getFitnessScore() << std::endl;
    std::cout << "transformation Matrix : " << std::endl << sac_ia.getFinalTransformation() << std::endl;

    start_icp=clock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(vg_source_cloud);
    icp.setInputTarget(vg_target_cloud);
    icp.setMaxCorrespondenceDistance(10.0);
    icp.align(*icp_cloud, sac_ia.getFinalTransformation());//重要，设置初始矩阵，相当于将sac ia 和icp连接起来了

    end_icp=clock();
    std::cerr << "calculate time is: " << float(end_icp - start_icp) / CLOCKS_PER_SEC <<"s"<< endl;
    std::cerr << std::endl << "ICP has" << (icp.hasConverged() ? "" : " not") << " converged, score : " << icp.getFitnessScore() << std::endl;
    std::cout << "transformation Matrix : " << std::endl << icp.getFinalTransformation() << std::endl;

    pcl::visualization::PCLVisualizer viewer("registration Viewer");
    int v1=0,v2=1,v3=2;
    viewer.createViewPort(0, 0.66, 1, 10.0, v1);
    viewer.createViewPort(0, 0.33, 1, 0.66, v2);
    viewer.createViewPort(0, 0.0, 1, 0.33, v3);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> vg_target_color(vg_target_cloud,255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> vg_source_color(vg_source_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sac_ia_color(sac_ia_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> icp_color(icp_cloud, 0, 255, 0);

    viewer.addPointCloud(vg_target_cloud, vg_target_color, "source cloud",v1);
    viewer.addPointCloud(vg_source_cloud, vg_source_color, "target1 cloud",v1);
    viewer.addPointCloud(vg_target_cloud, vg_target_color, "target2 cloud", v2);
    viewer.addPointCloud(sac_ia_cloud, sac_ia_color, "sac_ia cloud",v2);
    viewer.addPointCloud(vg_target_cloud, vg_target_color, "target3 cloud", v3);
    viewer.addPointCloud(icp_cloud, icp_color, "icp cloud",v3);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "source cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "target1 cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "target2 cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "sac_ia cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "target3 cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "icp cloud");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}
