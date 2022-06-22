#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace fgsp {

struct GraphManagerConfig {
  int verbose = 0u;  // Verbosity level (0:quiet)
  std::string map_frame = "map";

  std::vector<double> odom_noise_std = {0.01, 0.01, 0.01, 0.01,
                                        0.01, 0.01};  // rad,rad,rad,m,m,m;
  std::vector<double> relative_noise_std = {0.01, 0.01, 0.01, 0.01,
                                            0.01, 0.01};  // rad,rad,rad,m,m,m;
  std::vector<double> anchor_noise_std = {0.001, 0.001, 0.001, 0.001,
                                          0.001, 0.001};  // rad,rad,rad,m,m,m;

  std::string odom_topic = "/odometry";
  std::string anchor_topic = "/graph_client/anchor_nodes";
  std::string relative_topic = "/graph_client/relative_nodes";
  int update_interval_ms = 10000;  // Graph optimization interval (ms)

  // Extrinsic calibrations
  std::vector<double> T_O_B = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  std::vector<double> T_B_C = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0};

  static GraphManagerConfig* init(rclcpp::Node& node);
  auto isValid() const -> bool;
};

}  // namespace fgsp