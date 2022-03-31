#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct ICP_LM
{
    ICP_LM(int grid_size, int distance_threshold, int iterations);
    ~ICP_LM() = default;

    void align(pcl::PointCloud<pcl::PointXYZ>::Ptr &model, pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
               Eigen::Vector4f &centroid);

    int _grid_size;
    int _distance_threshold;
    int _iterations;
    Eigen::Matrix<float, 6, 1> _state;

    std::tuple<Eigen::Matrix3f, Eigen::Vector3f> state2rot();

    /**
     * http://planning.cs.uiuc.edu/node103.html
     *
     *  |r11 r12 r13 |
     *  |r21 r22 r23 |
     *  |r31 r32 r33 |
     *
     *  yaw: alpha=arctan(r21/r11)
     *  pitch: beta=arctan(-r31/sqrt( r32^2+r33^2 ) )
     *  roll: gamma=arctan(r32/r33)
     * @param R
     * @param t
     */
    void rot2state(const Eigen::Matrix3f R, const Eigen::Vector3f t);
};
