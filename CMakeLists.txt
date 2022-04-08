cmake_minimum_required(VERSION 3.0.2)
project(ICP)

set(CMAKE_CXX_STANDARD 17)

include(cmake/pkgs.cmake)
include_CATCH2()
include_fmt()

find_package(Ceres 2.1.0 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.8 REQUIRED)
find_package(Boost 1.54 COMPONENTS filesystem system program_options REQUIRED)

add_library(viewer src/viewer.cpp)
target_include_directories(viewer PRIVATE ${PCL_INCLUDE_DIRS} src)
target_link_libraries(viewer PUBLIC ${PCL_LIBRARIES})

add_library(icp_LM src/icp_LM.cpp src/viewer.cpp)
target_include_directories(icp_LM PRIVATE ${Eigen3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} src)
target_link_libraries(icp_LM PUBLIC fmt::fmt viewer ${CERES_LIBRARIES})

add_executable(pcd_alignment src/pcd_alignment.cpp)
target_include_directories(pcd_alignment PRIVATE ${PCL_INCLUDE_DIRS} src)
target_link_libraries(pcd_alignment PRIVATE icp_LM viewer)

add_executable(pcd_alignment_ceres src/pcd_alignment_ceres.cpp)
target_include_directories(pcd_alignment_ceres PRIVATE ${PCL_INCLUDE_DIRS} src)
target_link_libraries(pcd_alignment_ceres PRIVATE icp_LM viewer)

add_executable(continuous_pcd_alignment src/continuous_pcd_alignment.cpp)
target_include_directories(continuous_pcd_alignment PRIVATE ${PCL_INCLUDE_DIRS} src)
target_link_libraries(continuous_pcd_alignment PRIVATE icp_LM)

add_executable(extract_features src/extract_features.cpp)
target_include_directories(extract_features PRIVATE ${PCL_INCLUDE_DIRS} src)
target_link_libraries(extract_features PRIVATE viewer fmt::fmt)

add_executable(test test/test.cpp)
target_include_directories(test PRIVATE ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Catch2_INCLUDE_DIRS} src)
target_link_libraries(test PRIVATE Catch2::Catch2WithMain fmt::fmt ${PCL_LIBRARIES} icp_LM pcd_file)
