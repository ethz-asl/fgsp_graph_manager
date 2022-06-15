#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "graph_manager/graph_manager.h"
#include "graph_manager/graph_manager_config.h"
#include "graph_manager/graph_manager_node.h"

class GraphManagerNode : public rclcpp::Node {
 public:
  GraphManagerNode();
  virtual ~GraphManagerNode() = default;

 private:
  std::unique_ptr<fgsp::GraphManager> manager_;
  std::unique_ptr<fgsp::GraphManagerConfig> config_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};