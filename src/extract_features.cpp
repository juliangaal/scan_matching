#include "utils.h"
#include "viewer.h"

#include <fmt/printf.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

struct Feature
{
    Feature() : idx(0), curvature(0) {}
    Feature(size_t idx, float curvature) : idx(idx), curvature(curvature) {}
    size_t idx;
    float curvature;
};

float range(const pcl::PointXYZ& p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    file2pcd<pcl::PointXYZ>(orig_cloud, "/media/glumanda/ouster/128_foyer_1_stair/pcds/frame_100.pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*orig_cloud, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr surf_cloud(new pcl::PointCloud<pcl::PointXYZ>());


    auto width = 1024;
    auto height = 128;
    float edge_threshold = 0.5;
    float surf_threshold = 0.005;
    
    std::vector<Feature> features;
    features.resize(width * height);
    
    std::vector<int> label;
    label.resize(width * height);
    
    std::vector<bool> neighbor_picked;
    neighbor_picked.resize(width * height);
    
    // calculate curvature
    for (int u = 0; u < height; ++u)
    {
        for (int v = 5; v < width - 6; ++v)
        {
            size_t i = u * width + v;
            float diffX = cloud->points[i - 5].x + cloud->points[i - 4].x + cloud->points[i - 3].x + cloud->points[i - 2].x + cloud->points[i - 1].x - 10 * cloud->points[i].x + cloud->points[i + 1].x + cloud->points[i + 2].x + cloud->points[i + 3].x + cloud->points[i + 4].x + cloud->points[i + 5].x;
            float diffY = cloud->points[i - 5].y + cloud->points[i - 4].y + cloud->points[i - 3].y + cloud->points[i - 2].y + cloud->points[i - 1].y - 10 * cloud->points[i].y + cloud->points[i + 1].y + cloud->points[i + 2].y + cloud->points[i + 3].y + cloud->points[i + 4].y + cloud->points[i + 5].y;
            float diffZ = cloud->points[i - 5].z + cloud->points[i - 4].z + cloud->points[i - 3].z + cloud->points[i - 2].z + cloud->points[i - 1].z - 10 * cloud->points[i].z + cloud->points[i + 1].z + cloud->points[i + 2].z + cloud->points[i + 3].z + cloud->points[i + 4].z + cloud->points[i + 5].z;

            features[i] = Feature(i, diffX * diffX + diffY * diffY + diffZ * diffZ);
            neighbor_picked[i] = false;
            label[i] = 0;
        }
    }

    for (int u = 0; u < height; ++u)
    {
        for (int v = 5; v < width - 6; ++v)
        {
            // mark occluded points
            size_t i = u * width + v;
            float depth1 = range(cloud->points[i]);
            float depth2 = range(cloud->points[i + 1]);

            if (depth1 - depth2 > 0.3)
            {
                neighbor_picked[i - 5] = true;
                neighbor_picked[i - 4] = true;
                neighbor_picked[i - 3] = true;
                neighbor_picked[i - 2] = true;
                neighbor_picked[i - 1] = true;
                neighbor_picked[i] = true;

            }

            if (depth2 - depth1 > 0.3)
            {
                neighbor_picked[i + 1] = true;
                neighbor_picked[i + 2] = true;
                neighbor_picked[i + 3] = true;
                neighbor_picked[i + 4] = true;
                neighbor_picked[i + 5] = true;
                neighbor_picked[i + 6] = true;
            }

            // parallel beam
            float diff1 = std::abs(range(cloud->points[i - 1]) - range(cloud->points[i]));
            float diff2 = std::abs(range(cloud->points[i + 1]) - range(cloud->points[i]));

            if (diff1 > 0.02 * range(cloud->points[i]) && diff2 > 0.02 * range(cloud->points[i]))
            {
                neighbor_picked[i] = true;
            }
        }
    }

    for (int u = 0; u < height; ++u)
    {
        for (int v = 5; v < width - 6; v += (width / 6))
        {
            auto sp = u * width + v;
            auto ep = sp + width / 6;
            if (ep >= (u+1) * width)
            {
                ep = (u+1) * width - 6;
            }

            std::sort(features.begin()+sp, features.begin()+ep, [&](const auto& f, const auto& f2)
            {
                return f.curvature < f2.curvature;
            });

            int max_edge_features = 20;
            for (int k = ep; k >= sp; --k)
            {
                size_t idx = features[k].idx;
                float curvature = features[k].curvature;

                if (curvature > edge_threshold && !neighbor_picked[idx] && max_edge_features-- != 0)
                {
                    label[idx] = 1;
                    edge_cloud->push_back(cloud->points[idx]);

                    for (int l = 0; l < 5; l++)
                    {
                        neighbor_picked[idx + l] = true;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        neighbor_picked[idx + l] = true;
                    }
                }
            }

            int max_plane_features = 20;
            for (int k = sp; k < ep; ++k)
            {
                size_t idx = features[k].idx;
                float curvature = features[k].curvature;
                if (curvature < surf_threshold && !neighbor_picked[idx] && max_plane_features-- != 0)
                {
                    label[idx] = -1;

                    for (int l = 0; l <= 5; l++)
                    {
                        neighbor_picked[idx + l] = true;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        neighbor_picked[idx + l] = true;
                    }
                }

                if (label[idx] < 0)
                {
                    surf_cloud->push_back(cloud->points[idx]);
                }
            }
        }
    }

    fmt::print("Edges: {}, Surf Features: {}\n", edge_cloud->size(), surf_cloud->size());
    Viewer viewer("features");
    viewer.add_pointcloud("cloud", cloud, 0.5, 255, 255, 255);
    viewer.add_pointcloud("edges", edge_cloud, 3.0, 255, 0, 0);
    viewer.add_pointcloud("surfs", surf_cloud, 3.0, 255, 255, 0);
    viewer.show_viewer();

    return 0;
}

