#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "graph_manager/graph_manager.h"
#include "graph_manager/graph_manager_config.h"
#include "graph_manager/graph_manager_node.h"
#include "graph_manager/graph_manager_publisher.h"
#include "graph_manager/graph_manager_visualizer.h"

class GraphManagerNode : public rclcpp::Node {
 public:
  GraphManagerNode();
  virtual ~GraphManagerNode() = default;

 private:
  std::unique_ptr<fgsp::GraphManager> manager_;
  std::unique_ptr<fgsp::GraphManagerConfig> config_;
  std::unique_ptr<fgsp::GraphManagerPublisher> publisher_;
  std::unique_ptr<fgsp::GraphManagerVisualizer> visualizer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr anchor_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr relative_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      absolute_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};