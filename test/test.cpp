#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "utils.h"
#include "jacobian.h"

TEST_CASE("test", "tests")
{
    REQUIRE(0 == 0);
}

TEST_CASE("Jacobian", "jacobian")
{
    State state = State::Zero();
    state[2] = M_PI;
    
    auto [R, t] = state_2_rot_trans(state);
    pcl::Normal n(0, 0, 1);
    
    Eigen::Vector3f point(1, 1, 1);
    point = R * point;
    std::cout << point << "\n";
    std::cout << R << "\n";
//    auto J = jacobian_plane(point, n);
}