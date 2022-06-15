#include "graph_manager/graph_manager_node.h"

#include <functional>

#include "graph_manager/graph_manager_logger.h"

GraphManagerNode::GraphManagerNode()
    : rclcpp::Node("graph_manager") {
  auto& logger = fgsp::GraphManagerLogger::getInstance();
  logger.logInfo("Initializing Graph Manager...");

  config_.reset(fgsp::GraphManagerConfig::init(*this));
  manager_ = std::make_unique<fgsp::GraphManager>(*config_);

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