#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

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
  std::vector<double> absolute_noise_std = {
      0.001, 0.001, 0.001, 0.001, 0.001, 0.001};  // rad,rad,rad,m,m,m;

  std::string odom_topic = "/odometry";
  std::string anchor_topic = "/graph_client/anchor_nodes";
  std::string relative_topic = "/graph_client/relative_nodes";
  std::string absolute_topic = "/anymal/absolute_reference";
  int update_interval_ms = 10000;      // Graph optimization interval (ms)
  bool approximate_ts_lookup = false;  // Use approximate time lookup
  double ts_lookup_threshold_s = 0.1;  // Time lookup threshold (s)

  // Extrinsic calibrations
  std::vector<double> T_O_B = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  std::vector<double> T_B_A = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0};

  static GraphManagerConfig* init(rclcpp::Node& node);
  auto isValid() const -> bool;
};

}  // namespace fgsp