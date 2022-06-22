#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
      std::unordered_map<gtsam::Key, double> const& keyTimestampMap);

 private:
  bool createPoseMessage(
      gtsam::Pose3 const& pose,
      geometry_msgs::msg::PoseStamped* pose_msg) const;
  void publishOptimizedPath(
      gtsam::Values const& result,
      std::unordered_map<gtsam::Key, double> const& keyTimestampMap);
  void publishSubmapMarkers(
      gtsam::Values const& result,
      std::unordered_map<gtsam::Key, double> const& keyTimestampMap);

  GraphManagerConfig const& config_;
  GraphManagerPublisher& publisher_;

  visualization_msgs::msg::Marker
      submap_marker_msg_;  // Submap(Relative) contraint marker messages
  visualization_msgs::msg::Marker
      submap_parent_marker_msg_;  // Submap-Parent node contraint marker
                                  // messages
  nav_msgs::msg::Path optimized_path_msg_;
};

}  // namespace fgsp