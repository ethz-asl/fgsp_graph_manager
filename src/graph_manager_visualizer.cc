#include "graph_manager/graph_manager_visualizer.h"

#include <gtsam/inference/Symbol.h>

namespace fgsp {

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)

GraphManagerVisualizer::GraphManagerVisualizer(
    GraphManagerConfig const& config, GraphManagerPublisher& publisher)
    : config_(config), publisher_(publisher) {
  optimized_path_msg_.header.frame_id = "map";
}

void GraphManagerVisualizer::clear() {
  submap_marker_msg_.points.clear();
  submap_marker_msg_.header.stamp = publisher_.getTimeNow();
  submap_parent_marker_msg_.points.clear();
  submap_parent_marker_msg_.header.stamp = publisher_.getTimeNow();

  optimized_path_msg_.poses.clear();
  optimized_path_msg_.header.stamp = publisher_.getTimeNow();
}

void GraphManagerVisualizer::update(
    gtsam::Values const& result,
    std::unordered_map<gtsam::Key, double> const& keyTimestampMap) {
  publishOptimizedPath(result, keyTimestampMap);
}

bool GraphManagerVisualizer::createPoseMessage(
    gtsam::Pose3 const& pose, geometry_msgs::msg::PoseStamped* pose_msg) const {
  if (pose_msg == nullptr) {
    return false;
  }
  pose_msg->pose.position.x = pose.translation().x();
  pose_msg->pose.position.y = pose.translation().y();
  pose_msg->pose.position.z = pose.translation().z();
  pose_msg->pose.orientation.x = pose.rotation().toQuaternion().x();
  pose_msg->pose.orientation.y = pose.rotation().toQuaternion().y();
  pose_msg->pose.orientation.z = pose.rotation().toQuaternion().z();
  pose_msg->pose.orientation.w = pose.rotation().toQuaternion().w();

  return true;
}

void GraphManagerVisualizer::publishOptimizedPath(
    gtsam::Values const& result,
    std::unordered_map<gtsam::Key, double> const& keyTimestampMap) {
  // Loop through result values - Result in absolute frame
  const std::size_t n_result = result.size();
  for (std::size_t i = 0u; i < n_result; ++i) {
    gtsam::Pose3 T_G_B = result.at<gtsam::Pose3>(X(i));

    // Create pose message.
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = rclcpp::Time(keyTimestampMap.at(
        i));  // Publish each pose at timestamp corresponding to node in the
              // graph (Note: ts=0 in case of map lookup failure)
    createPoseMessage(T_G_B, &pose_msg);
    optimized_path_msg_.poses.emplace_back(pose_msg);
  }

  // Publish Path
  publisher_.publish(optimized_path_msg_, "/optimized_path");
}

}  // namespace fgsp
