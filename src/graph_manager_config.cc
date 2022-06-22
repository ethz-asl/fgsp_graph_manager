#include "graph_manager/graph_manager_config.h"
#include "graph_manager/graph_manager_logger.h"

namespace {

class ParameterParser {
 public:
  ParameterParser(rclcpp::Node& node) : node_(node) {}

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
  config->map_frame = parser.get_parameter("map_frame", config->map_frame);

  // Operation
  config->pos_delta = parser.get_parameter("pos_delta", config->pos_delta);
  config->rot_delta = parser.get_parameter("rot_delta", config->rot_delta);

  // Noise parameters
  config->odom_noise_std =
      parser.get_parameter("odom_noise_std", config->odom_noise_std);
  config->absolute_noise_std =
      parser.get_parameter("absolute_noise_std", config->absolute_noise_std);
  config->relative_noise_std =
      parser.get_parameter("relative_noise_std", config->relative_noise_std);
  config->anchor_noise_std =
      parser.get_parameter("anchor_noise_std", config->anchor_noise_std);

  // Topics
  config->odom_topic = parser.get_parameter("odom_topic", config->odom_topic);
  config->anchor_topic =
      parser.get_parameter("anchor_topic", config->anchor_topic);
  config->relative_topic =
      parser.get_parameter("relative_topic", config->relative_topic);
  config->update_interval_ms =
      parser.get_parameter("update_interval_ms", config->update_interval_ms);

  config->T_O_B = parser.get_parameter("T_O_B", config->T_O_B);
  config->T_B_C = parser.get_parameter("T_B_C", config->T_B_C);

  return config;
}

bool GraphManagerConfig::isValid() const {
  auto& logger = GraphManagerLogger::getInstance();
  if (odom_noise_std.size() != 6u) {
    logger.logError(
        "GraphManager - Odometry noise std vector has wrong size: " +
        std::to_string(odom_noise_std.size()));
    return false;
  }

  if (absolute_noise_std.size() != 6u) {
    logger.logError(
        "GraphManager - Absolute noise std vector has wrong size: " +
        std::to_string(absolute_noise_std.size()));
    return false;
  }

  if (relative_noise_std.size() != 6u) {
    logger.logError(
        "GraphManager - Absolute noise std vector has wrong size: " +
        std::to_string(relative_noise_std.size()));
    return false;
  }

  if (anchor_noise_std.size() != 6u) {
    logger.logError(
        "GraphManager - Absolute noise std vector has wrong size: " +
        std::to_string(anchor_noise_std.size()));
    return false;
  }

  if (T_O_B.size() != 16u) {
    logger.logError(
        "GraphManager - T_O_B vector has wrong size: " +
        std::to_string(T_O_B.size()));
    return false;
  }

  if (T_B_C.size() != 16u) {
    logger.logError(
        "GraphManager - T_B_C vector has wrong size: " +
        std::to_string(T_B_C.size()));
    return false;
  }

  return true;
}

}  // namespace fgsp
