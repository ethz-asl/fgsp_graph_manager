#include "graph_manager/graph_manager_node.h"
#include <chrono>
#include <exception>
#include <functional>

#include <std_msgs/msg/string.hpp>

#include "graph_manager/graph_manager_logger.h"

GraphManagerNode::GraphManagerNode()
    : rclcpp::Node("graph_manager") {
  auto& logger = fgsp::GraphManagerLogger::getInstance();
  logger.logInfo("Initializing Graph Manager...");

  config_.reset(fgsp::GraphManagerConfig::init(*this));
  if (!config_->isValid()) {
    logger.logError("Invalid configuration. Aborting.");
    return;
  }
  publisher_ = std::make_unique<fgsp::GraphManagerPublisher>(*this);
  if (publisher_ == nullptr) {
    logger.logError("Invalid publisher. Aborting.");
    return;
  }

  visualizer_ = std::make_unique<fgsp::GraphManagerVisualizer>(*config_, *publisher_);
  manager_ = std::make_unique<fgsp::GraphManager>(*config_, *publisher_, *visualizer_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      config_->odom_topic, 10, std::bind(&fgsp::GraphManager::odometryCallback, manager_.get(), std::placeholders::_1));

  anchor_sub_ = create_subscription<nav_msgs::msg::Path>(
      config_->anchor_topic, 10, std::bind(&fgsp::GraphManager::processAnchorConstraints, manager_.get(), std::placeholders::_1));

  relative_sub_ = create_subscription<nav_msgs::msg::Path>(
      config_->relative_topic, 10, std::bind(&fgsp::GraphManager::processRelativeConstraints, manager_.get(), std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::milliseconds(config_->update_interval_ms),
                             std::bind(&fgsp::GraphManager::updateGraphResults, manager_.get()));

  logger.logInfo("Subscribed input odometry to " + config_->odom_topic);
  logger.logInfo("Graph Manager initialized");
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto graph_manager_node = std::make_shared<GraphManagerNode>();
  rclcpp::spin(graph_manager_node);
  rclcpp::shutdown();
  return 0;
}