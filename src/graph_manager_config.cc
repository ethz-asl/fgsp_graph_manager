#include "graph_manager/graph_manager_config.h"

namespace {

class ParameterParser {
 public:
  ParameterParser(rclcpp::Node& node)
      : node_(node) {}

  template <typename T>
  T get_parameter(const std::string& name, T default_value) {
    node_.declare_parameter(name, default_value);
    T value;
    if (!node_.get_parameter(name, value)) {
      std::cerr << "Parameter " << name << " not found\n";
      return default_value;
    }
    return value;
  }

 private:
  rclcpp::Node& node_;
};
}  // namespace

namespace fgsp {

GraphManagerConfig* GraphManagerConfig::init(rclcpp::Node& node) {
  auto config = new GraphManagerConfig();
  auto parser = ::ParameterParser(node);

  // Frames
  config->verbose = parser.get_parameter("verbose", config->verbose);
  config->world_frame = parser.get_parameter("world_frame", config->world_frame);
  config->map_frame = parser.get_parameter("map_frame", config->map_frame);
  config->base_frame = parser.get_parameter("base_frame", config->base_frame);

  // Operation
  config->pos_delta = parser.get_parameter("pos_delta", config->pos_delta);
  config->rot_delta = parser.get_parameter("rot_delta", config->rot_delta);

  // Calibration
  config->lidar_frame = parser.get_parameter("lidar_frame", config->lidar_frame);
  config->camera_frame = parser.get_parameter("camera_frame", config->camera_frame);
  config->imu_frame = parser.get_parameter("imu_frame", config->imu_frame);

  // Noise parameters
  config->odom_noise_std = parser.get_parameter("odom_noise_std", config->odom_noise_std);
  config->absolute_noise_std = parser.get_parameter("absolute_noise_std", config->absolute_noise_std);
  config->submap_noise_std = parser.get_parameter("submap_noise_std", config->submap_noise_std);
  config->anchor_noise_std = parser.get_parameter("anchor_noise_std", config->anchor_noise_std);

  config->odom_topic = parser.get_parameter("odom_topic", config->odom_topic);

  return config;
}

}  // namespace fgsp
