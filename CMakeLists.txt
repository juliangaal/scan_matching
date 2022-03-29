cmake_minimum_required(VERSION 3.0.2)
project(ICP)

set(CMAKE_CXX_STANDARD 17)

include(cmake/pkgs.cmake)
include_CATCH2()
include_fmt()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.8 REQUIRED)
find_package(Boost 1.54 COMPONENTS filesystem system program_options REQUIRED)

add_library(icp_LM src/icp_LM.cpp src/viewer.cpp)
target_include_directories(icp_LM PRIVATE ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} src)
target_link_libraries(icp_LM PUBLIC fmt::fmt ${PCL_LIBRARIES})

add_executable(main src/main.cpp)
target_include_directories(icp_LM PRIVATE ${PCL_INCLUDE_DIRS} src)
target_link_libraries(main PRIVATE icp_LM)

add_executable(test test/test.cpp)
target_include_directories(test PRIVATE ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Catch2_INCLUDE_DIRS} src)
target_link_libraries(test PRIVATE Catch2::Catch2WithMain fmt::fmt ${PCL_LIBRARIES} icp_LM)
