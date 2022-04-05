#pragma once

#include <tuple>
#include <chrono>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using State = Eigen::Matrix<float, 6, 1>;
using Hessian = Eigen::Matrix<float, 6, 6>;
using Clock = std::chrono::steady_clock;

namespace Eigen
{
using Vector6f = Eigen::Matrix<float, 1, 6>;
}

namespace base
{
class NotImplementedException : public std::logic_error
{
public:
    NotImplementedException() : std::logic_error("Function not yet implemented.") {}
};
}

struct StopWatch
{
    StopWatch() : _start(Clock::now()) {}
    ~StopWatch() = default;
    
    template <typename T>
    double stop()
    {
        return std::chrono::duration_cast<T>(Clock::now() - _start).count();
    }
    
    void reset()
    {
        _start = Clock::now();
    }
    
    std::chrono::time_point<std::chrono::steady_clock> _start;
};

template <typename T>
inline void file2pcd(typename pcl::PointCloud<T>::Ptr& cloud, const std::string& filename)
{
    if (pcl::io::loadPCDFile<T>(filename, *cloud) == -1)
    {
        throw std::runtime_error("PCL IO Error");
    }
}

/**
 * Convert state (euler angles + translation vector)
 * to rotation matrix (right handed!) and translation vector
 * @return tuple of rotation matrix + translation vector
 */
inline std::tuple<Eigen::Matrix3f, Eigen::Vector3f> state_2_rot_trans(const State& state)
{
//    const auto& roll = state[0];
//    const auto& pitch = state[1];
//    const auto& yaw = state[2];
//    Eigen::Vector3f t(state[3], state[4], state[5]);
//
//    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
//    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
//    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
//
//    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
//    Eigen::Matrix3f R = q.matrix();

//    alpha, beta, gamma, x, y, z = euler[0], euler[1], euler[2], euler[3], euler[4], euler[5]
    const auto& alpha = state[0];
    const auto& beta = state[1];
    const auto& gamma = state[2];
    Eigen::Vector3f t(state[3], state[4], state[5]);

    Eigen::Matrix<float, 3, 3> Rot_z, Rot_y, Rot_x;
    Rot_z << std::cos(gamma), std::sin(gamma), 0, -std::sin(gamma), std::cos(gamma), 0, 0, 0, 1;
    Rot_y << std::cos(beta), 0, -std::sin(beta), 0, 1, 0, std::sin(beta), 0, std::cos(beta);
    Rot_x << 1, 0, 0, 0, std::cos(alpha), std::sin(alpha), 0, -std::sin(alpha),  std::cos(alpha);

    return { Rot_z * Rot_y * Rot_x, t };
}

/**
 * Convert rotation matrix + translation vector
 * to state (roll, pitch, yaw, translation vector)
 *
 * Source: http://planning.cs.uiuc.edu/node103.html
 *
 * |r11 r12 r13 |
 * |r21 r22 r23 |
 * |r31 r32 r33 |
 *
 * yaw: alpha=arctan(r21/r11)
 * pitch: beta=arctan(-r31/sqrt( r32^2+r33^2 ) )
 * roll: gamma=arctan(r32/r33)
 * @param state
 * @param R
 * @param t
 */
inline void rot_trans_2_state(State& state, const Eigen::Matrix3f& R, const Eigen::Vector3f& t)
{
//    state[0] = std::atan2(R(2,1),R(2,2));
//    state[1] = std::atan2(-R(2,0), std::pow( R(2,1)*R(2,1) +R(2,2)*R(2,2) ,0.5));
//    state[2] = std::atan2(R(1,0),R(0,0));
//    state[3] = t.x();
//    state[4] = t.y();
//    state[5] = t.z();

//    cs = np.linalg.norm(R[:2,0])
    float alpha, beta, gamma;
    auto cs = R.block<2,1>(0,0).norm();
    if (cs < 1e-16)
    {
        alpha = std::atan2(-R(1, 2), R(1, 1));
        beta = std::atan2(-R(1, 2), cs);
        gamma = 0;
    }
    else
    {
        alpha = std::atan2(R(2, 1), R(2, 2));
        beta = std::atan2(-R(2, 0), cs);
        gamma = std::atan2(R(1, 0), R(0, 0));
    }
    
    state[0] = alpha;
    state[1] = beta;
    state[2] = gamma;
    state[3] = t.x();
    state[4] = t.y();
    state[5] = t.z();
}

inline Eigen::Vector6f jacobian_plane(const Eigen::Vector3f& point, const Eigen::Vector3f& n)
{
    Eigen::Vector6f J = Eigen::Vector6f::Zero();
    J.block<1, 3>(0, 3) = n;
    J.block<1, 3>(0, 0) = -point.cross(n);
    return J;
}

inline Eigen::Vector6f jacobian_plane(const Eigen::Vector3f& point, const pcl::Normal& pcl_n)
{
    return jacobian_plane(point, pcl_n.getNormalVector3fMap());
}
