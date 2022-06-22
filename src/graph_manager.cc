#include "graph_manager/graph_manager.h"

#include <chrono>
#include <sstream>

#include <geometry_msgs/msg/pose_stamped.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "graph_manager/graph_manager_logger.h"

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)

namespace fgsp {

GraphManager::GraphManager(
    GraphManagerConfig const& config, GraphManagerPublisher& publisher,
    GraphManagerVisualizer& visualizer)
    : config_(config), publisher_(publisher), visualizer_(visualizer) {
  auto& logger = GraphManagerLogger::getInstance();
  logger.logInfo(
      "GraphManager - Verbosity level set to: " +
      std::to_string(config.verbose));
  logger.logInfo("GraphManager - Robot map frame set to: " + config.map_frame);

  odom_noise_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
      config.odom_noise_std.data());
  relative_noise_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
      config.relative_noise_std.data());
  anchor_noise_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
      config.anchor_noise_std.data());

  // TODO(lbern): Move to logger
  Eigen::IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");
  std::stringstream ss;
  ss << "GraphManager - Odometry factor noise: "
     << odom_noise_.transpose().format(clean_fmt) << "\n"
     << "GraphManager - Relative factor noise: "
     << relative_noise_.transpose().format(clean_fmt) << "\n"
     << "GraphManager - Anchor factor noise: "
     << anchor_noise_.transpose().format(clean_fmt) << "\n";
  logger.logInfo(ss.str());

  auto T_O_B =
      Eigen::Map<const Eigen::Matrix<double, 4, 4>>(config.T_O_B.data());
  T_O_B_ = gtsam::Pose3(
      gtsam::Rot3(T_O_B.block(0, 0, 3, 3)), T_O_B.block(0, 3, 3, 1));

  // TODO(lbern): Move to logger
  ss.clear();
  ss << "T_O_B: " << T_O_B_.matrix().format(clean_fmt) << "\n";
  logger.logInfo(ss.str());

  // Set factor graph params
  params_.optimizationParams = gtsam::ISAM2GaussNewtonParams();
  params_.factorization =
      gtsam::ISAM2Params::QR;  // CHOLESKY:Fast but non-stable //QR:Slower but
                               // more stable in poorly conditioned problems
  params_.relinearizeThreshold = 0.05;
  params_.enableRelinearization = true;
  params_.cacheLinearizedFactors = false;
  params_.relinearizeSkip = 1;
  params_.evaluateNonlinearError = false;
  params_.findUnusedFactorSlots =
      false;  // NOTE: Set false for factor removal lookup
  params_.enablePartialRelinearizationCheck = true;
  params_.enableDetailedResults = false;

  // Initialize factor graph object
  graph_ = std::make_shared<gtsam::ISAM2>(params_);
  graph_->params().print("Graph Integrator - Parameters:");
}

void GraphManager::odometryCallback(nav_msgs::msg::Odometry const& odom) {
  if (graph_ == nullptr) {
    const auto& logger = GraphManagerLogger::getInstance();
    logger.logError(
        "GraphManager - Graph is nullptr, cannot add odometry message");
    return;
  }
  // Current timestamp and odom pose
  const double ts = odom.header.stamp.sec * 1e9 + odom.header.stamp.nanosec;
  const auto& p = odom.pose.pose;
  const gtsam::Pose3 T_M_O(
      gtsam::Rot3(
          p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
      gtsam::Point3(p.position.x, p.position.y, p.position.z));

  // IMU pose
  gtsam::Pose3 T_M_B = T_M_O * T_O_B_;
  // Compute delta in IMU frame
  if (first_odom_msg_)  // Store first pose to compute zero delta incase input
                        // doesn't start from zero
    last_Base_pose_ = T_M_B;

  gtsam::Pose3 T_B1_B2 = last_Base_pose_.between(T_M_B);

  // Add delta pose factor to graph
  gtsam::Key new_key;
  auto t1 = std::chrono::high_resolution_clock::now();
  {
    // If first lidar odometry message has been received
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (first_odom_msg_) {
      // Initialize - Add temporary identity prior factor
      new_key = stateKey();
      addPriorFactor(new_key, gtsam::Pose3::identity());
      first_odom_msg_ = false;
    } else {
      // Update keys
      auto old_key = stateKey();
      new_key = newStateKey();
      // Calculate new IMU pose estimate in M
      gtsam::Pose3 T_M_B = T_M_B_inc_ * T_B1_B2;
      // Add delta and estimate as between factor
      addPoseBetweenFactor(old_key, new_key, T_B1_B2, T_M_B);
    }
    // Update Timestamp-Key & Key-Timestamp Map
    timestamp_key_map_[ts] = new_key;
    key_timestamp_map_[new_key] = ts;
    // Get updated result
    T_M_B_inc_ = graph_->calculateEstimate<gtsam::Pose3>(X(new_key));
  }
  auto t2 = std::chrono::high_resolution_clock::now();

  // Save last IMU pose for calculting delta
  last_Base_pose_ = T_M_B;

  if (config_.verbose > 0 && new_key % 10 == 0) {
    auto const& logger = GraphManagerLogger::getInstance();
    logger.logInfo(
        "\033[35mODOMETRY\033[0m ts: " + std::to_string(odom.header.stamp.sec) +
        ", key: " + std::to_string(new_key) + ", time(ms): " +
        std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                .count()));
  }
}

void GraphManager::processAnchorConstraints(nav_msgs::msg::Path const& path) {
  auto const& logger = GraphManagerLogger::getInstance();
  if (graph_ == nullptr) {
    const auto& logger = GraphManagerLogger::getInstance();
    logger.logError(
        "GraphManager - Graph is nullptr, cannot process anchor "
        "constraints.");
    return;
  }
  if (first_odom_msg_) {
    logger.logError("First odometry message has not been received yet!");
    return;
  }

  // Loop through all pose updates and add prior factor on graph nodes
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    auto t1 = std::chrono::high_resolution_clock::now();
    gtsam::FactorIndices remove_factor_idx;
    for (const geometry_msgs::msg::PoseStamped& pose_msg : path.poses) {
      // Update timestamp
      const double ts =
          pose_msg.header.stamp.sec * 1e9 + pose_msg.header.stamp.nanosec;

      // Find corresponding key in graph
      gtsam::Key key;
      auto key_itr = timestamp_key_map_.find(ts);
      if (key_itr != timestamp_key_map_.end()) {
        key = key_itr->second;  // Save Key
      } else {
        if (config_.verbose)
          logger.logInfo(
              "\033[36mANCHOR\033[0m - Found no closest key for ts: " +
              std::to_string(ts));
        continue;
      }

      if (key == 0)
        continue;

      // Create Update pose
      const auto& p = pose_msg.pose;
      const gtsam::Pose3 T_M_B(
          gtsam::Rot3(
              p.orientation.w, p.orientation.x, p.orientation.y,
              p.orientation.z),
          gtsam::Point3(p.position.x, p.position.y, p.position.z));

      // Compare if anchor pose on same key has changed
      auto anchor_itr = key_anchor_pose_map_.find(key);
      if (anchor_itr != key_anchor_pose_map_.end()) {
        bool equal = anchor_itr->second.equals(T_M_B, 0.2);
        if (equal) {
          if (config_.verbose)
            logger.logInfo(
                "\033[36mANCHOR\033[0m - Found Key: " + std::to_string(key) +
                ", ts: " + std::to_string(ts) +
                ", Equal: \033[32mYES\033[0m - SKIP");

          continue;
        }
      }

      // Find if Prior factor exists at Key and get Factor Index
      auto itr = key_anchor_factor_idx_map_.find(key);
      if (itr != key_anchor_factor_idx_map_.end()) {
        if (config_.verbose)
          logger.logInfo(
              "\033[36mANCHOR\033[0m - Found Key: " + std::to_string(key) +
              ", ts: " + std::to_string(ts) + ", Equal: \033[31mNO\033[0m" +
              ", Removing Prior with Index: " + std::to_string(itr->second));
        remove_factor_idx.push_back(itr->second);
      } else if (config_.verbose)
        logger.logInfo(
            "\033[36mANCHOR\033[0m - Found Key: " + std::to_string(key) +
            ", ts: " + std::to_string(ts) +
            ", Equal: \033[31mNO\033[0m - New Prior Added");

      // Update graph
      static auto anchorNoise =
          gtsam::noiseModel::Diagonal::Sigmas(anchor_noise_);
      new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          X(key), T_M_B, anchorNoise);
      graph_->update(new_factors_, gtsam::Values(), remove_factor_idx);
      new_factors_.resize(0);  // Reset
      incFactorCount();

      // Assosicate index of new prior factor to graph key
      updateKeyAnchorFactorIdxMap(key);
      // Associate Anchor pose to key
      updateKeyAnchorPoseMap(key, T_M_B);
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    if (config_.verbose)
      logger.logInfo(
          "\033[36mANCHOR-UPDATE\033[0m - time(ms): " +
          std::to_string(
              std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                  .count()));
  }
}

void GraphManager::processRelativeConstraints(nav_msgs::msg::Path const& path) {
  auto const& logger = GraphManagerLogger::getInstance();
  if (graph_ == nullptr) {
    const auto& logger = GraphManagerLogger::getInstance();
    logger.logError(
        "GraphManager - Graph is nullptr, cannot process relative "
        "constraints.");
    return;
  }
  if (first_odom_msg_) {
    logger.logInfo("GraphManager - Graph not initialized from odometry yet");
    return;
  }

  // Find corresponding parent key in graph
  const auto parent_ts =
      path.header.stamp.sec * 1e9 + path.header.stamp.nanosec;
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    auto parent_itr = timestamp_key_map_.find(parent_ts);
    if (parent_itr != timestamp_key_map_.end()) {
      gtsam::Key parent_key = parent_itr->second;
      // Loop through relative(parent)-relative(child) constraints
      auto t1 = std::chrono::high_resolution_clock::now();
      const std::size_t n_poses = path.poses.size();
      for (size_t i = 0; i < n_poses; ++i) {
        const auto child_ts = path.poses[i].header.stamp.sec * 1e9 +
                              path.poses[i].header.stamp.nanosec;

        // Check if child is associated to a key
        auto child_itr = timestamp_key_map_.find(child_ts);
        if (child_itr != timestamp_key_map_.end()) {
          gtsam::Key child_key = child_itr->second;
          // std::cout << std::fixed << ", Child Key : " << child_key << ", ts:
          // " << child_ts << std::endl;

          // Skip if keys are same
          if (child_key == parent_key) {
            if (config_.verbose > 0)
              logger.logInfo(
                  "GraphManager - Same relaitve parent/child keys, key:" +
                  std::to_string(child_key));
            continue;
          }

          // Check if a previous BetweenFactor exists between two keys
          gtsam::FactorIndices remove_factor_idx;
          int rmIdx = findRelativeFactorIdx(parent_key, child_key, true);
          if (rmIdx != -1)
            remove_factor_idx.push_back(rmIdx);

          // Add relative-relative BetweenFactor
          static auto relativeNoise =
              gtsam::noiseModel::Diagonal::Sigmas(relative_noise_);
          const auto& p = path.poses[i].pose;
          gtsam::Pose3 T_B1B2(
              gtsam::Rot3(
                  p.orientation.w, p.orientation.x, p.orientation.y,
                  p.orientation.z),
              gtsam::Point3(p.position.x, p.position.y, p.position.z));
          gtsam::BetweenFactor<gtsam::Pose3> relativeBF(
              X(parent_key), X(child_key), T_B1B2, relativeNoise);
          new_factors_.add(relativeBF);
          // Update Graph
          graph_->update(new_factors_, gtsam::Values(), remove_factor_idx);
          new_factors_.resize(0);
          incFactorCount();
          updateKeyRelativeFactorIdxMap(parent_key, child_key);
          if (config_.verbose > 0)
            logger.logInfo(
                "\033[34mRELATIVE\033[0m - : P(" + std::to_string(parent_key) +
                ")-C(" + std::to_string(child_key));
        } else
          continue;
      }
      auto t2 = std::chrono::high_resolution_clock::now();

      if (config_.verbose > 0)
        logger.logInfo(
            "\033[34mRELATIVE-UPDATE\033[0m - Constraints added: " +
            std::to_string(n_poses) + ", time(ms): " +
            std::to_string(
                std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                    .count()));
    } else {
      if (config_.verbose > 0)
        logger.logInfo(
            "\033[34mRELATIVE\033[0m  - Found no key for parent at ts: " +
            std::to_string(parent_ts) + " --- SKIPPING CHILDERN ---");
      return;
    }
  }
}

void GraphManager::addPriorFactor(
    const gtsam::Key key, const gtsam::Pose3& pose) {
  if (graph_ == nullptr) {
    const auto& logger = GraphManagerLogger::getInstance();
    logger.logError("GraphManager - Graph is nullptr, cannot add prior factor");
    return;
  }
  // Prior factor noise
  static auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);

  // Initial estimate - Use pose as estimate (For prior factors at graph init)
  gtsam::Values estimate;
  estimate.insert(X(key), pose);

  // Add prior factor and update graph
  new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(key), pose, priorNoise);
  graph_->update(new_factors_, estimate);
  new_factors_.resize(0);  // Reset
  incFactorCount();
}

void GraphManager::addPoseBetweenFactor(
    const gtsam::Key old_key, const gtsam::Key new_key,
    const gtsam::Pose3& pose_delta, const gtsam::Pose3& pose_estimate) {
  if (graph_ == nullptr) {
    const auto& logger = GraphManagerLogger::getInstance();
    logger.logError(
        "GraphManager - Graph is nullptr, cannot add pose between factor");
    return;
  }
  // Create Pose BetweenFactor
  static auto poseBFNoise = gtsam::noiseModel::Diagonal::Sigmas(odom_noise_);
  gtsam::BetweenFactor<gtsam::Pose3> poseBF(
      X(old_key), X(new_key), pose_delta, poseBFNoise);

  // Pose estimate
  gtsam::Values estimate;
  estimate.insert(X(new_key), pose_estimate);

  // Add factor and update graph
  new_factors_.add(poseBF);
  graph_->update(new_factors_, estimate);
  new_factors_.resize(0);  // Reset
  incFactorCount();
}

void GraphManager::updateGraphResults() {
  if (graph_ == nullptr) {
    const auto& logger = GraphManagerLogger::getInstance();
    logger.logError("GraphManager - Graph is nullptr, cannot update results.");
    return;
  }
  // Check if graph has initialized
  if (first_odom_msg_)
    return;

  // Update results
  gtsam::Values result;
  std::unordered_map<gtsam::Key, double> keyTimestampMap;
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    result = graph_->calculateBestEstimate();
    keyTimestampMap =
        key_timestamp_map_;  // copy cost 36000 elements(10Hz * 1Hr) ~10ms
  }

  // Visualize results
  visualizer_.clear();
  visualizer_.update(
      result, keyTimestampMap, relative_parent_child_key_map_,
      key_anchor_pose_map_);
}

void GraphManager::updateKeyRelativeFactorIdxMap(
    const gtsam::Key parent_key, const gtsam::Key child_key) {
  const std::size_t factor_idx = getFactorCount() - 1u;
  key_relative_factor_idx_map_[parent_key].emplace(factor_idx);
  key_relative_factor_idx_map_[child_key].emplace(factor_idx);

  // Save Parent-Child keys for visualization
  relative_parent_child_key_map_[parent_key].emplace(child_key);
}

int GraphManager::findRelativeFactorIdx(
    const gtsam::Key parent_key, const gtsam::Key child_key, bool erase) {
  // Get sets of constraint indices at graph keys
  auto& p_set = key_relative_factor_idx_map_[parent_key];
  auto& c_set = key_relative_factor_idx_map_[child_key];

  // Find common constraint index between two graph Keys i.e. a BetweenFactor
  // should have same factor index at two graph keys(nodes)
  std::set<std::size_t> result;
  std::set_intersection(
      p_set.begin(), p_set.end(), c_set.begin(), c_set.end(),
      std::inserter(result, result.begin()));

  // Check if success
  int output = -1;
  if (result.size() == 1) {
    output = *result.begin();

    // Remove common constraint indices from both sets
    if (erase) {
      p_set.erase(output);
      c_set.erase(output);
    }
  } else if (result.size() > 1) {
    GraphManagerLogger::getInstance().logInfo(
        "GraphManager - Relative constraint connected by mutliple between "
        "factors");
  }

  return output;
}

}  // namespace fgsp
