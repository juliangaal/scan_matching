cmake_minimum_required(VERSION 3.0.2)
project(ICP)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

add_compile_definitions(PARAMS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/params")

# find_package(Ceres 2.1.0 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.10.0 EXACT REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/ext/ext.cmake)

add_library(viewer src/viewer.cpp)
target_include_directories(viewer PRIVATE ${PCL_INCLUDE_DIRS} src)
target_link_libraries(viewer PUBLIC ${PCL_LIBRARIES})

add_executable(icp_lm src/icp_lm/main.cpp src/icp_lm/icp_lm.cpp)
target_include_directories(icp_lm PRIVATE ${PCL_INCLUDE_DIRS} src)
target_link_libraries(icp_lm PRIVATE fmt::fmt-header-only toml11 viewer Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_options(icp_lm PRIVATE -Wall -Weffc++ -Wpedantic -ffast-math -Wextra)

add_executable(sac_ia_gicp src/sac_ia_gicp/sac_ia_gicp.cpp)
target_include_directories(sac_ia_gicp PRIVATE src ext ${PCL_INCLUDE_DIRS})
target_link_libraries(sac_ia_gicp fmt::fmt-header-only Eigen3::Eigen toml11 ${PCL_LIBRARIES})
target_compile_options(sac_ia_gicp PRIVATE -Wall -Weffc++ -Wpedantic -ffast-math -Wextra)

# add_executable(pcd_alignment_ceres src/pcd_alignment_ceres.cpp)
# target_include_directories(pcd_alignment_ceres PRIVATE ${PCL_INCLUDE_DIRS} src)
# target_link_libraries(pcd_alignment_ceres PRIVATE icp_lm_lib viewer)

# add_executable(continuous_pcd_alignment src/pcd_alignment.cpp)
# target_include_directories(continuous_pcd_alignment PRIVATE ${PCL_INCLUDE_DIRS} src)
# target_link_libraries(continuous_pcd_alignment PRIVATE icp_lm_lib)

# add_executable(extract_features src/extract_features.cpp)
# target_include_directories(extract_features PRIVATE ${PCL_INCLUDE_DIRS} src)
# target_link_libraries(extract_features PRIVATE viewer fmt::fmt)

add_executable(test test/test.cpp)
target_include_directories(test PRIVATE ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Catch2_INCLUDE_DIRS} src)
target_link_libraries(test PRIVATE Catch2::Catch2WithMain fmt::fmt Eigen3::Eigen ${PCL_LIBRARIES})
