#pragma once

#include <tuple>
#include <Eigen/Dense>
#include <pcl/point_types.h>

using State = Eigen::Matrix<float, 6, 1>;


/**
 * Convert state (euler angles + translation vector)
 * to rotation matrix (right handed!) and translation vector
 * @return tuple of rotation matrix + translation vector
 */
std::tuple<Eigen::Matrix3f, Eigen::Vector3f> state_2_rot_trans(const State& state)
{
    const auto& roll = state[0];
    const auto& pitch = state[1];
    const auto& yaw = state[2];
    Eigen::Vector3f t(state[3], state[4], state[5]);

    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3f R = q.matrix();

    return { R, t };
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
void rot_trans_2_state(State& state, const Eigen::Matrix3f R, const Eigen::Vector3f t)
{
    state[0] = std::atan2(R(2,1),R(2,2));
    state[1] = std::atan2(-R(2,0), std::pow( R(2,1)*R(2,1) +R(2,2)*R(2,2) ,0.5));
    state[2] = std::atan2(R(1,0),R(0,0));
    state[3] = t.x();
    state[4] = t.y();
    state[5] = t.z();
}

Eigen::Matrix<float, 1, 6> jacobian_plane(const Eigen::Vector3f& point, const pcl::Normal& pcl_n)
{
    auto J = Eigen::Matrix<float, 1, 6>::Zero();
    auto n = pcl_n.getNormalVector3fMap();
    J.block<1, 3>(0, 3) = -point.cross(n);
    J.block<1, 3>(0, 0) = n;
    return J;
}
