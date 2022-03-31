#include "utils.h"
#include "pcd_file.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <fmt/ostream.h>
#include <fmt/printf.h>

TEST_CASE("Euler", "[euler]")
{
    State state = State::Zero();
    state[2] = M_PI;

    auto [R, t] = state_2_rot_trans(state);
    pcl::Normal n(0, 0, 1);

    Eigen::Vector3f point(1, 1, 1);
    point = R * point;
    std::cout << point << "\n";
    std::cout << R << "\n";
}

TEST_CASE("pcd_file", "[pcd_file]")
{
    int RING_SIZE = 5;
    std::vector<Eigen::Vector3f> ring1;
    std::vector<Eigen::Vector3f> ring2;
    std::vector<Eigen::Vector3f> ring3;
    ring1.resize(RING_SIZE);
    ring2.resize(RING_SIZE);
    ring3.resize(RING_SIZE);

    for (int i = 0; i < RING_SIZE; ++i)
    {
        ring1[i] = Eigen::Vector3f::Ones();
        ring2[i] = Eigen::Vector3f::Ones();
        ring3[i] = Eigen::Vector3f::Ones();
    }

    std::vector<std::vector<Eigen::Vector3f>> pcl;
    pcl.push_back(ring1);
    pcl.push_back(ring2);
    pcl.push_back(ring2);

    PCDFile file("../data/test.pcd");
    file.writePoints(pcl);
}

TEST_CASE("Jacobian", "[jacobian]")
{
    {
        Eigen::Vector6f result;
        result << -0, -0, -0, 1, 1, 1;

        auto point = Eigen::Vector3f::Ones();
        auto n = Eigen::Vector3f::Ones();
        auto pcl_n = pcl::Normal(n.x(), n.y(), n.z());

        auto J = jacobian_plane(point, n);
        REQUIRE(J.isApprox(result));

        J = jacobian_plane(point, pcl_n);
        REQUIRE(J.isApprox(result));
    }
    {
        Eigen::Vector6f result;
        result << -0, -0, -0, 0, 0, 0;

        auto point = Eigen::Vector3f::Ones();
        auto n = Eigen::Vector3f::Zero();
        auto pcl_n = pcl::Normal(n.x(), n.y(), n.z());

        auto J = jacobian_plane(point, n);
        REQUIRE(J.isApprox(result));

        J = jacobian_plane(point, pcl_n);
        REQUIRE(J.isApprox(result));
    }
    {
        Eigen::Vector6f result;
        result << 0, 0, -1, 0, 1, 0;

        auto point = Eigen::Vector3f(1, 0, 0);
        auto n = Eigen::Vector3f(0, 1, 0);
        auto pcl_n = pcl::Normal(n.x(), n.y(), n.z());

        auto J = jacobian_plane(point, n);
        REQUIRE(J.isApprox(result));

        J = jacobian_plane(point, pcl_n);
        REQUIRE(J.isApprox(result));
    }
    {
        Eigen::Vector6f result;
        result << 0, 1, 0, 0, 0, 1;

        auto point = Eigen::Vector3f(1, 0, 0);
        auto n = Eigen::Vector3f(0, 0, 1);
        auto pcl_n = pcl::Normal(n.x(), n.y(), n.z());

        auto J = jacobian_plane(point, n);
        REQUIRE(J.isApprox(result));

        J = jacobian_plane(point, pcl_n);
        REQUIRE(J.isApprox(result));
    }
    fmt::print("Jacobian tests successful\n");
}