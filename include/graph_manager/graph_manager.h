#pragma once

#include <algorithm>
#include <map>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#define SLOW_BUT_CORRECT_BETWEENFACTOR  // Increases accuracy in handling
                                        // rotations
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "graph_manager/graph_manager_config.h"
#include "graph_manager/graph_manager_publisher.h"
#include "graph_manager/graph_manager_visualizer.h"

namespace fgsp {

class GraphManager {
 public:
  explicit GraphManager(
      GraphManagerConfig const& config, GraphManagerPublisher& publisher,
      GraphManagerVisualizer& visualizer);

  void odometryCallback(nav_msgs::msg::Odometry const& odom);
  void absoluteCallback(
      geometry_msgs::msg::PoseWithCovarianceStamped const& pose_stamped);
  void bufferAnchorConstraints(nav_msgs::msg::Path const& path);
  void bufferRelativeConstraints(nav_msgs::msg::Path const& path);

  void processConstraints();
  void processAbsoluteConstraints(
      std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> const&
          constraints);
  void processAnchorConstraints(
      std::vector<nav_msgs::msg::Path> const& constraints);
  void processRelativeConstraints(
      std::vector<nav_msgs::msg::Path> const& constraints);

  // Get state key
  auto stateKey() const -> gtsam::Key { return state_key_; }
  // State key increment
  auto newStateKey() -> gtsam::Key { return ++state_key_; }
  // Add prior
  void addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose);
  // Add a pose between factor
  void addPoseBetweenFactor(
      const gtsam::Key old_key, const gtsam::Key new_key,
      const gtsam::Pose3& pose_delta, const gtsam::Pose3& pose_estimate);
  // Update ISAM graph
  void updateGraphResults();

  // Lookup maps for key-factor association
  void updateKeyAnchorFactorIdxMap(const gtsam::Key key) {
    key_anchor_factor_idx_map_[key] = factor_count_ - 1;
  }
  void updateKeyAnchorPoseMap(const gtsam::Key key, const gtsam::Pose3& pose) {
    key_anchor_pose_map_[key] = pose;
  }
  void updateKeyRelativeFactorIdxMap(
      const gtsam::Key parent_key, const gtsam::Key child_key);
  auto findRelativeFactorIdx(
      const gtsam::Key parent_key, const gtsam::Key child_key,
      bool erase = false) -> int;

  auto findExactKeyForTs(const double ts, gtsam::Key* key) const -> bool;
  auto findClosestKeyForTs(const double ts, gtsam::Key* key) const -> bool;

  // Factor count increment and retreivel
  void incFactorCount(std::size_t incr = 1u) { factor_count_ += incr; }
  auto getFactorCount() -> std::size_t { return factor_count_; }

  gtsam::Pose3 T_O_B_;      // Base(B) to Odometry(O)
  gtsam::Pose3 T_B_A_;      // Absolute Reference (A) to Odometry(O)
  gtsam::Pose3 T_G_M_;      // Global(B) to Map(M)
  gtsam::Pose3 T_M_B_inc_;  // Base(B) to Map(M) - incremental

  // Factor graph
  std::size_t factor_count_ = 0u;  // Total factors (existing + removed)
  std::mutex graph_mutex_;         // For adding new factors and graph update
  gtsam::ISAM2Params params_;      // Graph parameters
  gtsam::NonlinearFactorGraph new_factors_;  // Factors to be added to the graph
  std::shared_ptr<gtsam::ISAM2> graph_;      // iSAM2 GRAPH object
  gtsam::Key state_key_ = 0;                 // Current state key

  // Factor noise vectors - ORDER RPY(rad) - XYZ(meters)
  gtsam::Vector6 odom_noise_;      // Odometry BetweenFactor Noise
  gtsam::Vector6 relative_noise_;  // Relatve BetweenFactor Noise
  gtsam::Vector6 anchor_noise_;    // Anchor PriorFactor Noise
  gtsam::Vector6 absolute_noise_;  // Anchor PriorFactor Noise

  // Odometry factor
  bool first_odom_msg_ = true;
  bool first_absolute_pose_ = true;
  gtsam::Pose3 last_Base_pose_;

  // Lookup map objects for key-to-factorIndex associations
  std::map<double, gtsam::Key>
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

  void calculateStats(boost::shared_ptr<gtsam::ISAM2Clique> const& clique);

  GraphManagerConfig const& config_;
  GraphManagerPublisher& publisher_;
  GraphManagerVisualizer& visualizer_;
  bool is_odom_degenerated_ = false;

  std::mutex anchor_constraints_mutex_;
  std::mutex absolute_reference_mutex_;
  std::mutex relative_constraints_mutex_;
  std::vector<nav_msgs::msg::Path> anchor_constraints_buffer_;
  std::vector<nav_msgs::msg::Path> relative_constraints_buffer_;
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>
      absolute_reference_buffer_;
  std::vector<std::size_t> n_components_;
  std::vector<std::size_t> nnz_components_;
};

}  // namespace fgsp
