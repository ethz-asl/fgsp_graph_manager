#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace fgsp {

struct GraphManagerConfig {
  int verbose = 0u;  // Verbosity level of MaplabIntegrator (0:quiet)
  std::string map_frame = "map";

  double pos_delta = 4.0;  // Minimum delta position difference Norm to save new
                           // pointcoud (meters)
  double rot_delta = 0.3;  // Minimum delta rotation difference Norm to save new
                           // pointcoud (radians)

  std::vector<double> odom_noise_std = {0.01, 0.01, 0.01, 0.01,
                                        0.01, 0.01};  // rad,rad,rad,m,m,m;
  std::vector<double> relative_noise_std = {0.01, 0.01, 0.01, 0.01,
                                            0.01, 0.01};  // rad,rad,rad,m,m,m;
  std::vector<double> anchor_noise_std = {0.001, 0.001, 0.001, 0.001,
                                          0.001, 0.001};  // rad,rad,rad,m,m,m;

  std::string odom_topic = "/odometry";
  std::string anchor_topic = "/graph_client/anchor_nodes";
  std::string relative_topic = "/graph_client/relative_nodes";
  int update_interval_ms =
      10000;  // Update interval at which the graph is optimized (ms)

  // Extrinsic calibrations
  std::vector<double> T_O_B = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  std::vector<double> T_B_C = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0};

  static GraphManagerConfig* init(rclcpp::Node& node);
  bool isValid() const;
};

}  // namespace fgsp