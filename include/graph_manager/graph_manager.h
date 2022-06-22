#pragma once

#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// #include "visualization_msgs/Marker.h"
// #include "visualization_msgs/MarkerArray.h"

#define SLOW_BUT_CORRECT_BETWEENFACTOR  // increases accuracy in handling
                                        // rotations
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "graph_manager/graph_manager_config.h"
#include "graph_manager/graph_manager_publisher.h"
#include "graph_manager/graph_manager_visualizer.h"

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)

namespace fgsp {

class GraphManager {
 public:
  explicit GraphManager(
      GraphManagerConfig const& config, GraphManagerPublisher& publisher,
      GraphManagerVisualizer& visualizer);

  void odometryCallback(nav_msgs::msg::Odometry const& odom);
  void processAnchorConstraints(nav_msgs::msg::Path const& path);
  void processRelativeConstraints(nav_msgs::msg::Path const& path);

  // Get State key
  auto stateKey() const -> gtsam::Key { return state_key_; }
  // State Key increment
  auto newStateKey() -> gtsam::Key { return ++state_key_; }
  // Add prior
  void addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose);
  // Add a pose between factor
  void addPoseBetweenFactor(
      const gtsam::Key old_key, const gtsam::Key new_key,
      const gtsam::Pose3& pose_delta, const gtsam::Pose3& pose_estimate);
  //   //Update ISAM graph
  void updateGraphResults();

  //   //Utility methods
  bool createPoseMessage(
      const gtsam::Pose3& pose,
      geometry_msgs::msg::PoseStamped* pose_msg) const;

  //   //Lookup maps for key-factor association
  void updateKeyAnchorFactorIdxMap(const gtsam::Key key) {
    key_anchor_factor_idx_map_[key] = factor_count_ - 1;
  }
  void updateKeyAnchorPoseMap(const gtsam::Key key, const gtsam::Pose3& pose) {
    key_anchor_pose_map_[key] = pose;
  }
  void updateKeyRelativeFactorIdxMap(
      const gtsam::Key parent_key, const gtsam::Key child_key);
  int findRelativeFactorIdx(
      const gtsam::Key parent_key, const gtsam::Key child_key,
      bool erase = false);

  //   //Factor count increment and retreivel
  void incFactorCount() { ++factor_count_; }
  auto getFactorCount() -> std::size_t { return factor_count_; }

  gtsam::Pose3 T_O_B_;      // IMU(B) to Odometry(O)
  gtsam::Pose3 T_B_C_;      // Camera(C) to  IMU(B)
  gtsam::Pose3 T_G_B_inc_;  // IMU to Global(G) - incremental

  // Factor graph
  std::size_t factor_count_ =
      0;                       // Counter for Total factors (existing + removed)
  std::mutex graph_mutex_;     // For adding new factors and graph update
  gtsam::ISAM2Params params_;  // Graph parameters
  gtsam::NonlinearFactorGraph
      new_factors_;                      // New factors to be added to the graph
  std::shared_ptr<gtsam::ISAM2> graph_;  // iSAM2 GRAPH object
  gtsam::Key state_key_ = 0;             // Current state key

  // Factor noise vectors - ORDER RPY(rad) - XYZ(meters)
  gtsam::Vector6 odom_noise_;      // Odometry BetweenFactor Noise
  gtsam::Vector6 relative_noise_;  // Relatve BetweenFactor Noise
  gtsam::Vector6 anchor_noise_;    // Anchor PriorFactor Noise

  // Odometry factor
  bool first_odom_msg_ = true;
  gtsam::Pose3 last_IMU_pose_;

  // Lookup map objects for key-to-factorIndex associations
  std::unordered_map<double, gtsam::Key>
      timestamp_key_map_;  // Timestamp-Key map for lookup of keys corresponding
                           // to odometry timestamps
  std::unordered_map<gtsam::Key, double>
      key_timestamp_map_;  // Key-Timestamp map used for publishing graph node
                           // timestamps for path message publishing
  std::unordered_map<gtsam::Key, size_t>
      key_anchor_factor_idx_map_;  // Key-PriorFactorIndex map for lookup of
                                   // indices of prior factor add at key for
                                   // Anchor poses
  std::unordered_map<gtsam::Key, gtsam::Pose3>
      key_anchor_pose_map_;  // Key-AnchorPose map for lookup of applied anchor
                             // pose as prior factor at Key
  std::unordered_map<gtsam::Key, std::set<size_t>>
      key_relative_factor_idx_map_;  // Key-RelativeBetweenFactorIndex map for
                                     // lookup of indices of betweenfactor added
                                     // at key for relative constraints
  std::unordered_map<gtsam::Key, std::set<gtsam::Key>>
      relative_parent_child_key_map_;  // Parent-Child keys for visualization of
                                       // relative constrinats

  GraphManagerConfig const& config_;
  GraphManagerPublisher& publisher_;
  GraphManagerVisualizer& visualizer_;
  bool is_odom_degenerated_ = false;
};

}  // namespace fgsp
