# Scan Matching

Several Scan Matching methods with visualization. Adjust parameters for each in 

## Dependencies

* Eigen3
* PCL
* fmt
* Boost
* Catch2 (optional)

## Local

* Point-to-Plane Iterative Closes Points (ICP) solved with Levenberg-Marquardt optimization strategy. Includes visualization. [icp_lm](./src/icp/icp_lm.cpp)

## Global

* Sample Consensus Initial Alignment (SAC-IA) on Fast Point Feature Histogram (FPFH) Features + Generalized Iterative Closest Point (GICP) as described in [Point Cloud Registration Method Based on SAC-IA and NDT Fusion](http://www.jgg09.com/EN/abstract/abstract12346.shtml#). [sac_ia_gicp](./src/sac_ia_gicp/sac_ia_gicp.cpp)

## Credits

simseg: [original authors'](https://github.com/simeseg/icp) python implementation in [scripts directory](./scripts)
