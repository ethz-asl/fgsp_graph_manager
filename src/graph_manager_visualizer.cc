#include "graph_manager/graph_manager_visualizer.h"

#include <gtsam/inference/Symbol.h>

namespace fgsp {

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)

GraphManagerVisualizer::GraphManagerVisualizer(
    GraphManagerConfig const& config, GraphManagerPublisher& publisher)
    : config_(config), publisher_(publisher) {
  anchor_marker_msg_.header.frame_id = config_.map_frame;
  anchor_marker_msg_.ns = "anchor_constraints";
  anchor_marker_msg_.type = visualization_msgs::msg::Marker::CUBE_LIST;
  anchor_marker_msg_.action = visualization_msgs::msg::Marker::ADD;
  anchor_marker_msg_.scale.x = anchor_marker_msg_.scale.y =
      anchor_marker_msg_.scale.z = 0.25f;
  anchor_marker_msg_.color.r = 0.0f;
  anchor_marker_msg_.color.g = 1.0f;
  anchor_marker_msg_.color.b = 1.0f;
  anchor_marker_msg_.color.a = 1.0f;
  anchor_marker_msg_.pose.orientation.w = 1.0;

  relative_marker_msg_ = anchor_marker_msg_;
  relative_marker_msg_.ns = "relative_constraints";
  relative_marker_msg_.type = visualization_msgs::msg::Marker::LINE_LIST;
  relative_marker_msg_.scale.x = 0.05f;

  relative_parent_marker_msg_ = anchor_marker_msg_;
  relative_parent_marker_msg_.ns = "relative_parent_nodes";
  relative_parent_marker_msg_.type =
      visualization_msgs::msg::Marker::SPHERE_LIST;

  relative_text_marker_msg_ = anchor_marker_msg_;
  relative_text_marker_msg_.ns = "relative_num_children";
  relative_text_marker_msg_.type =
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  relative_text_marker_msg_.color.a = relative_text_marker_msg_.color.r =
      relative_text_marker_msg_.color.g = relative_text_marker_msg_.color.b =
          1.0f;

  // Init frames
  optimized_path_msg_.header.frame_id = config_.map_frame;
  relative_marker_msg_.header.frame_id = config_.map_frame;
  relative_parent_marker_msg_.header.frame_id = config_.map_frame;
  relative_text_marker_msg_.header.frame_id = config_.map_frame;
}

void GraphManagerVisualizer::clear() {
  relative_marker_msg_.points.clear();
  relative_marker_msg_.colors.clear();
  relative_marker_msg_.header.stamp = publisher_.getTimeNow();
  relative_parent_marker_msg_.points.clear();
  relative_parent_marker_msg_.colors.clear();
  relative_parent_marker_msg_.header.stamp = publisher_.getTimeNow();

  relative_text_marker_msg_.text.clear();
  relative_text_marker_msg_.header.stamp = publisher_.getTimeNow();
  relative_text_marker_array_msg_.markers.clear();

  anchor_marker_msg_.points.clear();
  anchor_marker_msg_.header.stamp = publisher_.getTimeNow();

  optimized_path_msg_.poses.clear();
  optimized_path_msg_.header.stamp = publisher_.getTimeNow();
}

void GraphManagerVisualizer::update(
    gtsam::Values const& result,
    std::unordered_map<gtsam::Key, double> const& key_timestamp_map,
    std::unordered_map<gtsam::Key, std::set<gtsam::Key>> const&
        relative_parent_map,
    std::unordered_map<gtsam::Key, gtsam::Pose3> const& key_anchor_pose_map) {
  publishOptimizedPath(result, key_timestamp_map);
  publishRelativeMarkers(result, relative_parent_map);
  publishAnchorMarkers(result, key_anchor_pose_map);
}

auto GraphManagerVisualizer::createPoseMessage(
    gtsam::Pose3 const& pose, geometry_msgs::msg::PoseStamped* pose_msg) const
    -> bool {
  if (pose_msg == nullptr) {
    return false;
  }
  pose_msg->header.frame_id = config_.map_frame;
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
    std::unordered_map<gtsam::Key, double> const& key_timestamp_map) {
  std::size_t const n_result = result.size();
  for (std::size_t i = 0u; i < n_result; ++i) {
    gtsam::Pose3 T_G_B = result.at<gtsam::Pose3>(X(i));

    // Create pose message.
    geometry_msgs::msg::PoseStamped pose_msg;

    // Publish each pose at timestamp corresponding to node in the
    // graph (Note: ts=0 in case of map lookup failure)
    pose_msg.header.stamp = rclcpp::Time(key_timestamp_map.at(i));
    pose_msg.header.frame_id = "map";
    createPoseMessage(T_G_B, &pose_msg);
    optimized_path_msg_.poses.emplace_back(pose_msg);
  }

  // Publish Path
  publisher_.publish(optimized_path_msg_, "/optimized_path");
}

void GraphManagerVisualizer::publishRelativeMarkers(
    gtsam::Values const& result,
    std::unordered_map<gtsam::Key, std::set<gtsam::Key>> const&
        relative_parent_map) {
  std::size_t const n_result = result.size();

  auto const result_ts = publisher_.getTimeNow();

  // Add the parent markers and their text labels from relative constraints.
  // Afterwards intererate through all children per parent and add them too.
  for (std::size_t i = 0u; i < n_result; ++i) {
    auto const parent_itr = relative_parent_map.find(i);
    if (parent_itr == relative_parent_map.cend()) {
      continue;
    }
    gtsam::Pose3 const parent_pose = result.at<gtsam::Pose3>(X(i));
    geometry_msgs::msg::Point parent_pt;
    parent_pt.x = parent_pose.translation().x();
    parent_pt.y = parent_pose.translation().y();
    parent_pt.z = parent_pose.translation().z();
    auto const color = createColor(i);
    relative_parent_marker_msg_.points.emplace_back(parent_pt);
    relative_parent_marker_msg_.colors.emplace_back(color);

    relative_text_marker_msg_.header.stamp = result_ts;
    relative_text_marker_msg_.id = i;
    relative_text_marker_msg_.pose.position.x = parent_pt.x;
    relative_text_marker_msg_.pose.position.y = parent_pt.y;
    relative_text_marker_msg_.pose.position.z = parent_pt.z + 0.25f;
    relative_text_marker_msg_.text = std::to_string(parent_itr->second.size());
    relative_text_marker_array_msg_.markers.emplace_back(
        relative_text_marker_msg_);

    // Loop through children
    for (auto const& childKey : parent_itr->second) {
      gtsam::Pose3 child_pose = result.at<gtsam::Pose3>(X(childKey));
      geometry_msgs::msg::Point child_pt;
      child_pt.x = child_pose.translation().x();
      child_pt.y = child_pose.translation().y();
      child_pt.z = child_pose.translation().z();

      // Add parent and child points to marker msg
      relative_marker_msg_.points.emplace_back(parent_pt);
      relative_marker_msg_.points.emplace_back(child_pt);
      relative_marker_msg_.colors.emplace_back(color);
      relative_marker_msg_.colors.emplace_back(color);
    }
  }
  publisher_.publish(relative_marker_msg_, "/constraint_markers");
  publisher_.publish(relative_parent_marker_msg_, "/constraint_markers");
  publisher_.publish(
      relative_text_marker_array_msg_, "/constraint_text_markers");
}

void GraphManagerVisualizer::publishAnchorMarkers(
    gtsam::Values const& result,
    std::unordered_map<gtsam::Key, gtsam::Pose3> const& key_anchor_pose_map) {
  std::size_t const n_result = result.size();
  for (std::size_t i = 0u; i < n_result; ++i) {
    auto const anchor_itr = key_anchor_pose_map.find(i);
    if (anchor_itr == key_anchor_pose_map.cend()) {
      continue;
    }
    geometry_msgs::msg::Point p;
    p.x = anchor_itr->second.translation().x();
    p.y = anchor_itr->second.translation().y();
    p.z = anchor_itr->second.translation().z();
    anchor_marker_msg_.points.emplace_back(p);
  }
  publisher_.publish(anchor_marker_msg_, "/constraint_markers");
}

auto GraphManagerVisualizer::createColor(
    std::size_t const idx, std::size_t max_idx) const
    -> std_msgs::msg::ColorRGBA {
  if (idx > max_idx)
    max_idx = max_idx * 2.0f;
  float const val =
      static_cast<float>(idx) * 255.0f / static_cast<float>(max_idx);

  std_msgs::msg::ColorRGBA color;
  color.r =
      std::round(std::sin(0.024f * val + 0.0f) * 127.0f + 128.0f) / 255.0f;
  color.g =
      std::round(std::sin(0.024f * val + 2.0f) * 127.0f + 128.0f) / 255.0f;
  color.b =
      std::round(std::sin(0.024f * val + 4.0f) * 127.0f + 128.0f) / 255.0f;
  color.a = 1.0f;
  return color;
}
}  // namespace fgsp
