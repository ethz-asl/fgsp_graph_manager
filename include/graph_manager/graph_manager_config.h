#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace fgsp {

struct GraphManagerConfig {
  int verbose = 0u;  // Verbosity level of MaplabIntegrator (0:quiet)
  std::string world_frame = "world";
  std::string map_frame = "map";
  std::string base_frame = "imu";
  std::string lidar_frame = "";   // LiDAR frame name - used for LiDAR-to-Sensor transform lookup
  std::string camera_frame = "";  // Frame of camera used for apriltag detection (absolute poses)
  std::string imu_frame = "";     // Frame of IMU used by maplab

  double pos_delta = 4.0;  // Minimum delta position difference Norm to save new pointcoud (meters)
  double rot_delta = 0.3;  // Minimum delta rotation difference Norm to save new pointcoud (radians)

  std::vector<double> odom_noise_std = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};          // rad,rad,rad,m,m,m;
  std::vector<double> absolute_noise_std = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};      // rad,rad,rad,m,m,m;
  std::vector<double> submap_noise_std = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};        // rad,rad,rad,m,m,m;
  std::vector<double> anchor_noise_std = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};  // rad,rad,rad,m,m,m;

  std::string odom_topic = "/odometry";
  std::string anchor_topic = "/graph_client/anchor_nodes";
  std::string relative_topic = "/graph_client/relative_nodes";
  int update_interval_ms = 10000;  // Update interval at which the graph is optimized (ms)

  // Extrinsic calibrations
  std::vector<double> T_O_B = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  std::vector<double> T_B_C = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};

  static GraphManagerConfig* init(rclcpp::Node& node);
  bool isValid() const;
};

}  // namespace fgsp