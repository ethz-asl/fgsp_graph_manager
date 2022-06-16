#include "graph_manager/graph_manager.h"

#include <chrono>
#include <sstream>

#include "graph_manager/graph_manager_logger.h"

namespace fgsp {

GraphManager::GraphManager(GraphManagerConfig const& config,
                           GraphManagerPublisher& publisher)
    : config_(config), publisher_(publisher) {
  auto& logger = GraphManagerLogger::getInstance();
  logger.logInfo("GraphManager - Verbosity level set to: " + std::to_string(config.verbose));
  logger.logInfo("GraphManager - World/Global frame set to: " + config.world_frame);
  logger.logInfo("GraphManager - Robot Map frame set to: " + config.map_frame);
  logger.logInfo("GraphManager - Robot Base frame set to: " + config.base_frame);

  logger.logInfo("GraphManager - Minimum Position(m) delta Norm: " + std::to_string(config.pos_delta));
  logger.logInfo("GraphManager - Minimum Rotation(rad) delta Norm: " + std::to_string(config.rot_delta));

  logger.logInfo("GraphManager - LiDAR Frame: " + config.lidar_frame);
  logger.logInfo("GraphManager - Camera Frame: " + config.camera_frame);
  logger.logInfo("GraphManager - IMU Frame: " + config.imu_frame);

  odomNoise_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(config.odom_noise_std.data());
  absoluteNoise_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(config.absolute_noise_std.data());
  submapNoise_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(config.submap_noise_std.data());
  anchorNoise_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(config.anchor_noise_std.data());

  // TODO(lbern): Move to logger
  Eigen::IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");
  std::stringstream ss;
  ss << "GraphManager - Odometry Factor Noise: " << odomNoise_.transpose().format(clean_fmt) << "\n"
     << "GraphManager - Absolute Factor Noise: " << absoluteNoise_.transpose().format(clean_fmt) << "\n"
     << "GraphManager - Submap Factor Noise: " << submapNoise_.transpose().format(clean_fmt) << "\n"
     << "GraphManager - Anchor Factor Noise: " << anchorNoise_.transpose().format(clean_fmt) << "\n";
  logger.logInfo(ss.str());

  auto T_O_B = Eigen::Map<const Eigen::Matrix<double, 4, 4>>(config.T_O_B.data());
  T_O_B_ = gtsam::Pose3(gtsam::Rot3(T_O_B.block(0, 0, 3, 3)), T_O_B.block(0, 3, 3, 1));

  auto T_B_C = Eigen::Map<const Eigen::Matrix<double, 4, 4>>(config.T_B_C.data());
  T_B_C_ = gtsam::Pose3(gtsam::Rot3(T_B_C.block(0, 0, 3, 3)), T_B_C.block(0, 3, 3, 1));

  // TODO(lbern): Move to logger
  ss.clear();
  ss << "T_O_B: " << T_O_B_.matrix().format(clean_fmt) << "\n"
     << "T_B_C: " << T_B_C_.matrix().format(clean_fmt) << "\n";
  logger.logInfo(ss.str());

  // Set factor graph params
  params_.optimizationParams = gtsam::ISAM2GaussNewtonParams();
  params_.factorization = gtsam::ISAM2Params::QR;  // CHOLESKY:Fast but non-stable //QR:Slower but more stable in poorly conditioned problems
  gtsam::FastMap<char, gtsam::Vector> relinTh;     // Set graph relinearization thresholds - must be lower case letters, check:gtsam::symbol_shorthand
  // params_.relinearizeThreshold = 0.05;
  // params_.setEnableRelinearization(true);
  // params_.setRelinearizeSkip(1);
  // params_.setCacheLinearizedFactors(false);
  // params_.setEvaluateNonlinearError(false);
  // params_.findUnusedFactorSlots = false;  // NOTE: Set false for factor removal lookup
  // params_.setEnablePartialRelinearizationCheck(true);
  // params_.setEnableDetailedResults(false);

  // Initialize factor graph object
  graph_ = std::make_shared<gtsam::ISAM2>(params_);
  graph_->params().print("Graph Integrator - Parameters:");
}

void GraphManager::odometryCallback(nav_msgs::msg::Odometry const& odom) {
  // Current timestamp and odom pose
  const double ts = odom.header.stamp.sec;
  const auto& p = odom.pose.pose;
  const gtsam::Pose3 T_M_O(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                           gtsam::Point3(p.position.x, p.position.y, p.position.z));

  // IMU pose
  gtsam::Pose3 T_M_B = T_M_O * T_O_B_;
  // Compute delta in IMU frame
  if (first_odom_msg_)  // Store first pose to compute zero delta incase input doesn't start from zero
    last_IMU_pose_ = T_M_B;

  gtsam::Pose3 T_B1_B2 = last_IMU_pose_.between(T_M_B);

  // Add delta pose factor to graph
  gtsam::Key new_key;
  auto t1 = std::chrono::high_resolution_clock::now();
  {
    // If first lidar odometry message has been received
    std::lock_guard<std::mutex> lock(graphMutex_);
    if (first_odom_msg_) {
      // Initialize - Add temporary identity prior factor
      new_key = stateKey();
      addPriorFactor(new_key, gtsam::Pose3::identity());
      first_odom_msg_ = false;
    } else {
      // Update keys
      auto old_key = stateKey();
      new_key = newStateKey();
      // Calculate new IMU pose estimate in G
      gtsam::Pose3 T_G_B = T_G_B_inc_ * T_B1_B2;
      // Add delta and estimate as between factor
      addPoseBetweenFactor(old_key, new_key, T_B1_B2, T_G_B);
    }
    // Update Timestamp-Key & Key-Timestamp Map
    timestampKeyMap_[ts] = new_key;
    keyTimestampMap_[new_key] = ts;
    // Get updated result
    T_G_B_inc_ = graph_->calculateEstimate<gtsam::Pose3>(X(new_key));
  }
  auto t2 = std::chrono::high_resolution_clock::now();

  // Save last IMU pose for calculting delta
  last_IMU_pose_ = T_M_B;

  // Check motion difference to know if incoming cloud should be saved
  gtsam::Pose3 cloud_delta = last_odom_save_pose_.between(T_M_O);
  if (cloud_delta.translation().norm() > config_.pos_delta ||
      cloud_delta.rotation().rpy().norm() > config_.rot_delta) {
    last_odom_save_pose_ = T_M_O;
  }

  // Publish Graph path
  // Pose Message
  geometry_msgs::msg::PoseStamped pose_msg;
  // pose_msg.header.frame_id = (is_odom_degenerated_ ? "degenerate" : "");
  pose_msg.header.frame_id = "map";
  pose_msg.header.stamp = odom.header.stamp;
  createPoseMessage(T_G_B_inc_, &pose_msg);

  // Node Message
  path_msg_.header.frame_id = "map";
  path_msg_.header.stamp = pose_msg.header.stamp;
  path_msg_.poses.emplace_back(std::move(pose_msg));

  // Publish Path
  publisher_.publish(path_msg_, "/incremental_path");

  if (config_.verbose > 0 && new_key % 10 == 0) {
    auto const& logger = GraphManagerLogger::getInstance();
    logger.logInfo("\033[35mODOMETRY\033[0m ts: " + std::to_string(odom.header.stamp.sec) + ", key: " + std::to_string(new_key) +
                   ", key: " + std::to_string(new_key) +
                   ", time(ms): " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()));
  }
}

// // void MaplabIntegrator::maplabAnchorCallback(const nav_msgs::Path::ConstPtr& pathPtr) {
// //   // Check if empty message
// //   if (pathPtr == nullptr || _firstOdomMsg) {
// //     std::cout << std::fixed << "\033[31mANCHOR\033[0m - Passed nullptr or graph not initialized from odometry yet" << std::endl;
// //     return;
// //   }

// //   // Loop through all pose updates and add prior factor on graph nodes
// //   {
// //     std::lock_guard<std::mutex> lock(_graphMutex);
// //     auto t1 = std::chrono::high_resolution_clock::now();
// //     gtsam::FactorIndices removeFactorIdx;
// //     for (const geometry_msgs::PoseStamped& pose_msg : pathPtr->poses) {
// //       // Update timestamp
// //       const double ts = pose_msg.header.stamp.toSec();

// //       // Find corresponding key in graph
// //       gtsam::Key key;
// //       auto key_itr = _timestampKeyMap.find(ts);
// //       if (key_itr != _timestampKeyMap.end()) {
// //         key = key_itr->second;  // Save Key
// //       } else {
// //         if (_verbose) logger.logInfo("\033[36mANCHOR\033[0m - Found no closest key for ts: :" << ros::Time(ts));
// //         continue;
// //       }

// //       // Only Absolute poses can be attached to Key 0
// //       if (key == 0)
// //         continue;

// //       // Create Update pose
// //       const auto& p = pose_msg.pose;
// //       const gtsam::Pose3 T_G_B(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
// //                                gtsam::Point3(p.position.x, p.position.y, p.position.z));

// //       // Compare if anchor pose on same key has changed
// //       auto anchor_itr = _keyAnchorPoseMap.find(key);
// //       if (anchor_itr != _keyAnchorPoseMap.end()) {
// //         bool equal = anchor_itr->second.equals(T_G_B, 0.2);
// //         if (equal) {
// //           if (_verbose) logger.logInfo("\033[36mANCHOR\033[0m - Found Key: " << key
// //                                                                               << ", ts: " << ros::Time(ts)
// //                                                                               << ", t(x,y,z): " << T_G_B.translation().transpose()
// //                                                                               << ", Equal: \033[32mYES\033[0m - SKIP");

// //           continue;
// //         }
// //       }

// //       // Find if Prior factor exists at Key and get Factor Index
// //       auto itr = _keyAnchorFactorIdxMap.find(key);
// //       if (itr != _keyAnchorFactorIdxMap.end()) {
// //         if (_verbose) logger.logInfo("\033[36mANCHOR\033[0m - Found Key: " << key
// //                                                                             << ", ts: " << ros::Time(ts)
// //                                                                             << ", Equal: \033[31mNO\033[0m"
// //                                                                             << ", Removing Prior with Index: " << itr->second);
// //         removeFactorIdx.push_back(itr->second);
// //       } else if (_verbose)
// //         logger.logInfo("\033[36mANCHOR\033[0m - Found Key: " << key
// //                                                               << ", ts: " << ros::Time(ts)
// //                                                               << ", t(x,y,z): " << T_G_B.translation().transpose()
// //                                                               << ", Equal: \033[31mNO\033[0m - New Prior Added");

// //       // Update graph
// //       static auto anchorNoise = gtsam::noiseModel::Diagonal::Sigmas(_anchorNoise);
// //       _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), T_G_B, anchorNoise);
// //       _graph->update(_newFactors, gtsam::Values(), removeFactorIdx);
// //       _newFactors.resize(0);  // Reset
// //       incFactorCount();

// //       // Update lookup maps
// //       updateKeyAnchorFactorIdxMap(key);    // Assosicate index of new prior factor to graph key
// //       updateKeyAnchorPoseMap(key, T_G_B);  // Associate Anchor pose to key
// //     }
// //     auto t2 = std::chrono::high_resolution_clock::now();
// //     if (_verbose) logger.logInfo("\033[36mANCHOR-UPDATE\033[0m - time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
// //   }
// // }

// // void MaplabIntegrator::maplabSubmapCallback(const nav_msgs::Path::ConstPtr& pathPtr) {
// //   // Check if empty message
// //   if (pathPtr == nullptr || _firstOdomMsg) {
// //     logger.logInfo("MaplabIntegrator - nullptr passed to Submap callback or graph not initialized from odometry yet");
// //     return;
// //   }

// //   // Find corresponding parent key in graph
// //   const auto& parent_ts = pathPtr->header.stamp.toSec();
// //   {
// //     std::lock_guard<std::mutex> lock(_graphMutex);
// //     auto parent_itr = _timestampKeyMap.find(parent_ts);
// //     if (parent_itr != _timestampKeyMap.end()) {
// //       gtsam::Key parent_key = parent_itr->second;
// //       // Loop through submap(parent)-submap(child) constraints
// //       auto t1 = std::chrono::high_resolution_clock::now();
// //       for (size_t i = 0; i < pathPtr->poses.size(); ++i) {
// //         const auto& child_ts = pathPtr->poses[i].header.stamp.toSec();
// //         // Check if child is associated to a key
// //         auto child_itr = _timestampKeyMap.find(child_ts);
// //         if (child_itr != _timestampKeyMap.end()) {
// //           gtsam::Key child_key = child_itr->second;
// //           // std::cout << std::fixed << ", Child Key : " << child_key << ", ts: " << child_ts << std::endl;

// //           // Skip if keys are same
// //           if (child_key == parent_key) {
// //             if (_verbose) logger.logInfo("MaplabIntegrator - Same Submap Parent/Child keys, key:" << child_key);
// //             continue;
// //           }

// //           // Check if a previous BetweenFactor exists between two keys
// //           gtsam::FactorIndices removeFactorIdx;
// //           int rmIdx = findSubmapFactorIdx(parent_key, child_key, true);
// //           if (rmIdx != -1)
// //             removeFactorIdx.push_back(rmIdx);

// //           // Add submap-submap BetweenFactor
// //           static auto submapNoise = gtsam::noiseModel::Diagonal::Sigmas(_submapNoise);
// //           const auto& p = pathPtr->poses[i].pose;
// //           gtsam::Pose3 T_B1B2(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
// //                               gtsam::Point3(p.position.x, p.position.y, p.position.z));
// //           gtsam::BetweenFactor<gtsam::Pose3> submapBF(X(parent_key), X(child_key), T_B1B2, submapNoise);
// //           _newFactors.add(submapBF);
// //           // Update Graph
// //           _graph->update(_newFactors, gtsam::Values(), removeFactorIdx);
// //           _newFactors.resize(0);
// //           incFactorCount();
// //           updateKeySubmapFactorIdxMap(parent_key, child_key);
// //           if (_verbose) logger.logInfo("\033[34mSUBMAP\033[0m - " << (rmIdx == -1 ? "Added" : "Replaced") << ": P(" << parent_key << ")-C(" << child_key << "), delta_t(x,y,z):" << T_B1B2.translation().transpose());
// //         } else
// //           continue;
// //       }
// //       auto t2 = std::chrono::high_resolution_clock::now();

// //       // DEBUG
// //       if (_verbose) logger.logInfo("\033[34mSUBMAP-UPDATE\033[0m - Constraints added: " << pathPtr->poses.size() << ", time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
// //       // DEBUG
// //     } else {
// //       if (_verbose) logger.logInfo("\033[34mSUBMAP\033[0m  - Found no key for parent at ts: " << ros::Time(parent_ts) << " --- SKIPPING CHILDERN ---");
// //       return;
// //     }
// //   }
// // }

void GraphManager::addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose) {
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
  newFactors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), pose, priorNoise);
  graph_->update(newFactors_, estimate);
  newFactors_.resize(0);  // Reset
  incFactorCount();
}

void GraphManager::addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose_delta, const gtsam::Pose3& pose_estimate) {
  if (graph_ == nullptr) {
    const auto& logger = GraphManagerLogger::getInstance();
    logger.logError("GraphManager - Graph is nullptr, cannot add pose between factor");
    return;
  }
  // Create Pose BetweenFactor
  static auto poseBFNoise = gtsam::noiseModel::Diagonal::Sigmas(odomNoise_);
  gtsam::BetweenFactor<gtsam::Pose3> poseBF(X(old_key), X(new_key), pose_delta, poseBFNoise);

  // Pose estimate
  gtsam::Values estimate;
  estimate.insert(X(new_key), pose_estimate);

  // Add factor and update graph
  newFactors_.add(poseBF);
  graph_->update(newFactors_, estimate);
  newFactors_.resize(0);  // Reset
  incFactorCount();
}

void GraphManager::updateGraphResults() {
  // Check if graph has initialized
  if (first_odom_msg_)
    return;

  // Update results
  gtsam::Values result;
  std::unordered_map<gtsam::Key, double> keyTimestampMap;
  auto t_0 = std::chrono::high_resolution_clock::now();
  {
    std::lock_guard<std::mutex> lock(graphMutex_);
    result = graph_->calculateBestEstimate();
    keyTimestampMap = keyTimestampMap_;  // copy cost 36000 elements(10Hz * 1Hr) ~10ms
  }
  auto t_update = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_0).count();

  // Publish result at now() timestamp to account for optmization delay
  // //   ros::Time resultTimestamp = ros::Time::now();

  // Clear constraint marker messages
  // //   _anchorMarkerMsg.header.stamp = resultTimestamp;
  // _anchorMarkerMsg.points.clear();
  // //   _absoluteMarkerMsg.header.stamp = resultTimestamp;
  // _absoluteMarkerMsg.points.clear();
  // //   _submapMarkerMsg.points.clear();
  // //   _submapMarkerMsg.colors.clear();
  // //   _submapMarkerMsg.header.stamp = resultTimestamp;
  // //   _submapParentMarkerMsg.points.clear();
  // //   _submapParentMarkerMsg.colors.clear();
  // //   _submapParentMarkerMsg.header.stamp = resultTimestamp;
  // //   _submapTextMarkerArrayMsg.markers.clear();

  auto t1 = std::chrono::high_resolution_clock::now();
  nav_msgs::msg::Path path_msg;

  // Loop through result values - Result in absolute frame
  T_G_M_ = result.at<gtsam::Pose3>(X(0));
  const std::size_t n_result = result.size();
  for (std::size_t i = 0; i < n_result; ++i) {
    gtsam::Pose3 T_G_B = result.at<gtsam::Pose3>(X(i));

    // Create pose message.
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = config_.world_frame;
    pose_msg.header.stamp = rclcpp::Time(keyTimestampMap[i]);  // Publish each pose at timestamp corresponding to node in the graph (Note: ts=0 in case of map lookup failure)
    createPoseMessage(T_G_B, &pose_msg);
    path_msg.poses.emplace_back(pose_msg);
  }

  // Publish Path
  // if (_pubUpdatedPath.getNumSubscribers() > 0) {
  //     pathMsg.header.frame_id = _world_frame;
  //     pathMsg.header.stamp = resultTimestamp;
  //     _pubUpdatedPath.publish(pathMsg);
  //   }

  // Update last optimzed pose
  T_G_B_opt_ = result.at<gtsam::Pose3>(X(result.size() - 1));

  auto t2 = std::chrono::high_resolution_clock::now();
  if (config_.verbose > 1) {
    auto logger = GraphManagerLogger::getInstance();
    logger.logInfo("\033[36mGRAPH UPDATE\033[0m - time(us):" + std::to_string(t_update) + ", Map Build(ms): " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()));
    logger.logInfo("\033[36mGRAPH UPDATE\033[0m - number of factors: " + std::to_string(factor_count_));
  }
}

bool GraphManager::createPoseMessage(const gtsam::Pose3& pose, geometry_msgs::msg::PoseStamped* pose_msg) const {
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

// // void MaplabIntegrator::updateKeySubmapFactorIdxMap(const gtsam::Key parent_key, const gtsam::Key child_key) {
// //   size_t factor_idx = getFactorCount() - 1;
// //   _keySubmapFactorIdxMap[parent_key].emplace(factor_idx);
// //   _keySubmapFactorIdxMap[child_key].emplace(factor_idx);

// //   // Save Parent-Child keys for visualization
// //   _submapParentChildKeyMap[parent_key].emplace(child_key);
// //   // DEBUG
// //   //  std::cout << "\033[34mSUBMAP\033[0m NEW Factor added at Index: " << factor_idx << ", between keys: " << parent_key << "/" << child_key << std::endl;
// // }

// // int MaplabIntegrator::findSubmapFactorIdx(const gtsam::Key parent_key, const gtsam::Key child_key, bool erase) {
// //   // Get sets of constraint indices at graph keys
// //   auto& p_set = _keySubmapFactorIdxMap[parent_key];
// //   auto& c_set = _keySubmapFactorIdxMap[child_key];

// //   // Find common constraint index between two graph Keys i.e. a BetweenFactor should have same factor index at two graph keys(nodes)
// //   std::set<size_t> result;
// //   std::set_intersection(p_set.begin(), p_set.end(),
// //                         c_set.begin(), c_set.end(),
// //                         std::inserter(result, result.begin()));

// //   // Check if success
// //   int output = -1;
// //   if (result.size() == 1) {
// //     output = *result.begin();

// //     // Remove common constraint indices from both sets
// //     if (erase) {
// //       p_set.erase(output);
// //       c_set.erase(output);
// //     }
// //   } else if (result.size() > 1) {
// //     logger.logInfo("MaplabIntegrator - Submaps connected by mutliple between factors");
// //     std::cout << "Indices found: ";
// //     for (auto& e : result)
// //       std::cout << e << " ";
// //     std::cout << ", between keys: " << parent_key << "/" << child_key << std::endl;
// //   }

// //   return output;
// // }

}  // namespace fgsp
