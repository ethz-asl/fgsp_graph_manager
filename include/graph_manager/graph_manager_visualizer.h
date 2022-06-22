#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "graph_manager/graph_manager_config.h"
#include "graph_manager/graph_manager_publisher.h"

namespace fgsp {

class GraphManagerVisualizer {
 public:
  explicit GraphManagerVisualizer(
      GraphManagerConfig const& config, GraphManagerPublisher& publisher);

  void clear();
  void update(
      gtsam::Values const& result,
      std::unordered_map<gtsam::Key, double> const& key_timestamp_map,
      std::unordered_map<gtsam::Key, std::set<gtsam::Key>> const&
          relative_parent_map,
      std::unordered_map<gtsam::Key, gtsam::Pose3> const& key_anchor_pose_map);

 private:
  auto createPoseMessage(
      gtsam::Pose3 const& pose, geometry_msgs::msg::PoseStamped* pose_msg) const
      -> bool;
  void publishOptimizedPath(
      gtsam::Values const& result,
      std::unordered_map<gtsam::Key, double> const& key_timestamp_map);
  void publishRelativeMarkers(
      gtsam::Values const& result,
      std::unordered_map<gtsam::Key, std::set<gtsam::Key>> const&
          relative_parent_map);
  void publishAnchorMarkers(
      gtsam::Values const& result,
      std::unordered_map<gtsam::Key, gtsam::Pose3> const& key_anchor_pose_map);
  auto createColor(std::size_t const idx, std::size_t max_idx = 5000u) const
      -> std_msgs::msg::ColorRGBA;

  GraphManagerConfig const& config_;
  GraphManagerPublisher& publisher_;

  visualization_msgs::msg::Marker
      relative_marker_msg_;  // Relative contraint marker
  visualization_msgs::msg::Marker
      relative_parent_marker_msg_;  // Relative-Parent node contraint marker
  visualization_msgs::msg::Marker
      relative_text_marker_msg_;  // Constraint text marker message
  visualization_msgs::msg::MarkerArray
      relative_text_marker_array_msg_;  // Constraint text marker message array
  visualization_msgs::msg::Marker
      anchor_marker_msg_;  // Anchor contraint marker

  nav_msgs::msg::Path optimized_path_msg_;
};

}  // namespace fgsp