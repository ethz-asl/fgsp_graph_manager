#include "graph_manager/graph_manager_node.h"
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

  manager_ = std::make_unique<fgsp::GraphManager>(*config_, *publisher_);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      config_->odom_topic, 10, std::bind(&fgsp::GraphManager::odometryCallback, manager_.get(), std::placeholders::_1));

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