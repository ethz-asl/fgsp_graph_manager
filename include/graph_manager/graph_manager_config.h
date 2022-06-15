#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace fgsp {

template <typename T>
T get_parameter(rclcpp::Node& node, const std::string& name, T default_value) {
  node.declare_parameter(name, default_value);
  T value;
  if (!node.get_parameter(name, value)) {
    std::cerr << "Parameter " << name << " not found\n";
    return default_value;
  }
  return value;
}

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

  static GraphManagerConfig*
  init(rclcpp::Node& node) {
    auto config = new GraphManagerConfig();
    // Frames
    config->verbose = get_parameter(node, "verbose", config->verbose);
    config->world_frame = get_parameter(node, "world_frame", config->world_frame);
    config->map_frame = get_parameter(node, "map_frame", config->map_frame);
    config->base_frame = get_parameter(node, "base_frame", config->base_frame);

    // Operation
    config->pos_delta = get_parameter(node, "pos_delta", config->pos_delta);
    config->rot_delta = get_parameter(node, "rot_delta", config->rot_delta);

    // Calibration
    config->lidar_frame = get_parameter(node, "lidar_frame", config->lidar_frame);
    config->camera_frame = get_parameter(node, "camera_frame", config->camera_frame);
    config->imu_frame = get_parameter(node, "imu_frame", config->imu_frame);

    // Noise parameters
    config->odom_noise_std = get_parameter(node, "odom_noise_std", config->odom_noise_std);
    config->absolute_noise_std = get_parameter(node, "absolute_noise_std", config->absolute_noise_std);
    config->submap_noise_std = get_parameter(node, "submap_noise_std", config->submap_noise_std);
    config->anchor_noise_std = get_parameter(node, "anchor_noise_std", config->anchor_noise_std);

    config->odom_topic = get_parameter(node, "odom_topic", config->odom_topic);

    return config;
  }
};

}  // namespace fgsp