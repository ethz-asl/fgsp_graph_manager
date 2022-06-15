#include "graph_manager/graph_manager.h"

namespace fgsp {

static void log_info(std::string&& msg) {
  std::cout << msg << std::endl;
}

static void log_error(const std::string& msg) {
  std::cerr << msg << std::endl;
}

GraphManager::GraphManager(GraphManagerConfig const& config)
    : config_(config) {
}
// // bool GraphManager::setup() {
// // // Parameters
// // if (privateNode.getParam("maplabIntegratorVerbosity", _verbose))
// //   log_info("MaplabIntegrator - Verbosity level set to: " << _verbose);
// // // Frames
// // if (privateNode.getParam("world_frame", _world_frame))
// //   log_info("MaplabIntegrator - World/Global frame set to: " << _world_frame);
// // if (privateNode.getParam("map_frame", _map_frame))
// //   log_info("MaplabIntegrator - Robot Map frame set to: " << _map_frame);
// // if (privateNode.getParam("base_frame", _base_frame))
// //   log_info("MaplabIntegrator - Robot Base frame set to: " << _base_frame);

// // // Pointcloud save delta
// // if (privateNode.getParam("cloudSavePosDelta", _cloudSavePosDelta))
// //   log_info("MaplabIntegrator - Minimum Position(m) delta Norm for saving pointcloud: " << _cloudSavePosDelta);
// // if (privateNode.getParam("cloudSaveRotDelta", _cloudSaveRotDelta))
// //   log_info("MaplabIntegrator - Minimum Rotation(rad) delta Norm for saving pointcloud: " << _cloudSaveRotDelta);

// // // Extrinsic Calibrations
// // // LiDAR frame for look-up
// // if (privateNode.getParam("lidarFrame", _lidar_frame))
// //   log_info("MaplabIntegrator - LiDAR Frame: " << _lidar_frame);
// // if (_lidar_frame.empty()) log_error("MaplabIntegrator - No LiDAR frame provided")
// // // Camera frame for look-up
// // if (privateNode.getParam("maplab_camera_frame", _camera_frame))
// //   log_info("MaplabIntegrator - Camera Frame: " << _camera_frame);
// // if (_camera_frame.empty().empty()) log_error("MaplabIntegrator - No Camera frame provided")
// // // IMU frame for look-up
// // if (privateNode.getParam("maplab_imu_frame", _imu_frame))
// //   log_info("MaplabIntegrator - IMU Frame: " << _imu_frame);
// // if (_imu_frame.empty()) log_error("MaplabIntegrator - No IMU frame provided")

// // // Factor Noise parameters
// // std::vector<double> odomNoise;
// // if (privateNode.getParam("odometryFactorNoise", odomNoise)) {
// //   _odomNoise << odomNoise[0], odomNoise[1], odomNoise[2], odomNoise[3], odomNoise[4], odomNoise[5];  // rad,rad,rad,m,m,m
// //   std::cout << "MaplabIntegrator - Odometry Factor Noise: " << _odomNoise.transpose() << std::endl;
// // } else {
// //   log_info("MaplabIntegrator - Odometry Factor Noise not defined");
// //   return false;
// // }
// // std::vector<double> absNoise;
// // if (privateNode.getParam("absoluteFactorNoise", absNoise)) {
// //   _absoluteNoise << absNoise[0], absNoise[1], absNoise[2], absNoise[3], absNoise[4], absNoise[5];  // rad,rad,rad,m,m,m
// //   std::cout << "MaplabIntegrator - Absolute Factor Noise: " << _absoluteNoise.transpose() << std::endl;
// // } else {
// //   log_info("MaplabIntegrator - Absolute Factor Noise not defined");
// //   return false;
// // }
// // std::vector<double> submapNoise;
// // if (privateNode.getParam("submapFactorNoise", submapNoise)) {
// //   _submapNoise << submapNoise[0], submapNoise[1], submapNoise[2], submapNoise[3], submapNoise[4], submapNoise[5];  // rad,rad,rad,m,m,m
// //   std::cout << "MaplabIntegrator - Submap Factor Noise: " << _submapNoise.transpose() << std::endl;
// // } else {
// //   log_info("MaplabIntegrator - Submap Factor Noise not defined");
// //   return false;
// // }
// // std::vector<double> anchorNoise;
// // if (privateNode.getParam("anchorFactorNoise", anchorNoise)) {
// //   _anchorNoise << anchorNoise[0], anchorNoise[1], anchorNoise[2], anchorNoise[3], anchorNoise[4], anchorNoise[5];  // rad,rad,rad,m,m,m
// //   std::cout << "MaplabIntegrator - Anchor Factor Noise: " << _anchorNoise.transpose() << std::endl;
// // } else {
// //   log_info("MaplabIntegrator - Anchor Factor Noise not defined");
// //   return false;
// // }
// // // Full graph result update interval
// // privateNode.getParam("updateResultsInterval", _updateResultsInterval);
// // log_info("MaplabIntegrator - Full Graph Result Updated Every: " << _updateResultsInterval << " seconds");

// // // Subscribers

// // // --------------------- NORMAL MODE ------------------------------------------
// // // Synced subscriber
// // _subOdom.subscribe(node, "odometry", 100);
// // _subOdomStatus.subscribe(node, "odometry_status", 100);
// // _syncPtr.reset(new message_filters::Synchronizer<_syncPolicy>(_syncPolicy(100), _subOdom, _subOdomStatus));
// // _syncPtr->registerCallback(boost::bind(&MaplabIntegrator::syncCallbackHandler, this, _1, _2, _3));
// // // --------------------- NORMAL MODE ------------------------------------------

// // // --------------------- EUROC ONLY ------------------------------------------
// // //_subOdomEUROC = node.subscribe<nav_msgs::Odometry>("odometry", 100, &MaplabIntegrator::lidarOdomCallbackEUROC, this);
// // // --------------------- EUROC ONLY ------------------------------------------

// // _subAbsolutePose = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("absolute_constraints", 100, &MaplabIntegrator::absolutePoseCallback, this);
// // _subMaplabAnchor = node.subscribe<nav_msgs::Path>("anchor_constraints", 100, &MaplabIntegrator::maplabAnchorCallback, this);
// // _subMaplabSubmap = node.subscribe<nav_msgs::Path>("submap_constraints", 3000, &MaplabIntegrator::maplabSubmapCallback, this);

// // // Publisher
// // _pubIncrementalPath = node.advertise<nav_msgs::Path>("/incremental_path", 10);
// // _pubUpdatedPath = node.advertise<nav_msgs::Path>("/optimized_path", 10);
// // _pathMsg.poses.reserve(40000);  // 1 hour path @ 10Hz ~ 36000
// // _pubIncPoseStamped = node.advertise<geometry_msgs::PoseStamped>("/incremental_pose", 10);
// // _pubOptPoseStamped = node.advertise<geometry_msgs::PoseStamped>("/optimized_pose", 10);
// // _pubConstraintMarkers = node.advertise<visualization_msgs::Marker>("/constraint_markers", 10);
// // _pubConstraintTextMarkers = node.advertise<visualization_msgs::MarkerArray>("/constraint_text_markers", 10);

// // // Timer-callback for graph update
// // _timerUpdateResults = node.createTimer(ros::Duration(_updateResultsInterval), std::bind(&MaplabIntegrator::updateGraphResults, this));

// // // Set factor graph params
// // _params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
// // _params.factorization = gtsam::ISAM2Params::QR;  // CHOLESKY:Fast but non-stable //QR:Slower but more stable in poorly conditioned problems
// // gtsam::FastMap<char, gtsam::Vector> relinTh;     // Set graph relinearization thresholds - must be lower case letters, check:gtsam::symbol_shorthand
// // relinTh['x'] = (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished();
// // _params.relinearizeThreshold = relinTh;
// // _params.setEnableRelinearization(true);
// // _params.setRelinearizeSkip(1);
// // _params.setCacheLinearizedFactors(false);
// // _params.setEvaluateNonlinearError(false);
// // _params.findUnusedFactorSlots = false;  // NOTE: Set false for factor removal lookup
// // _params.setEnablePartialRelinearizationCheck(true);
// // _params.setEnableDetailedResults(false);
// // // Initialize factor graph object
// // _graph = std::make_shared<gtsam::ISAM2>(_params);
// // _graph->params().print("Graph Integrator - Parameters:");

// // // Initialize Anchor Marker types
// // _anchorMarkerMsg.header.frame_id = _world_frame;
// // _anchorMarkerMsg.ns = "anchor_constraints";
// // _anchorMarkerMsg.type = visualization_msgs::Marker::CUBE_LIST;
// // _anchorMarkerMsg.action = visualization_msgs::Marker::ADD;
// // _anchorMarkerMsg.scale.x = _anchorMarkerMsg.scale.y = _anchorMarkerMsg.scale.z = 0.25f;
// // _anchorMarkerMsg.color.r = 0.0f;
// // _anchorMarkerMsg.color.g = 1.0f;
// // _anchorMarkerMsg.color.b = 1.0f;
// // _anchorMarkerMsg.color.a = 1.0f;
// // _anchorMarkerMsg.pose.orientation.w = 1.0;
// // // Initialize Absolute Marker types
// // _absoluteMarkerMsg = _anchorMarkerMsg;
// // _absoluteMarkerMsg.ns = "absolute_constraints";
// // _absoluteMarkerMsg.color.r = 1.0f;
// // _absoluteMarkerMsg.color.g = 1.0f;
// // _absoluteMarkerMsg.color.b = 0.0f;
// // // Initialize Submap Marker types
// // // Relative line markers between Parent and Child nodes
// // _submapMarkerMsg = _anchorMarkerMsg;
// // _submapMarkerMsg.ns = "submap_relative_constraints";
// // _submapMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
// // _submapMarkerMsg.scale.x = 0.05f;
// // // Parent nodes sphere markers
// // _submapParentMarkerMsg = _anchorMarkerMsg;
// // _submapParentMarkerMsg.ns = "submap_parent_nodes";
// // _submapParentMarkerMsg.type = visualization_msgs::Marker::SPHERE_LIST;
// // // Parent constraint count text marker
// // _submapTextMarkerMsg = _anchorMarkerMsg;
// // _submapTextMarkerMsg.ns = "submap_num_children";
// // _submapTextMarkerMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
// // _submapTextMarkerMsg.color.a = _submapTextMarkerMsg.color.r = _submapTextMarkerMsg.color.g = _submapTextMarkerMsg.color.b = 1.0f;

// // return true;
// }

// // void MaplabIntegrator::syncCallbackHandler(const nav_msgs::OdometryConstPtr& odomPtr,
// //                                            const OptStatusConstPtr& odomStatusPtr,
// //                                            const sensor_msgs::PointCloud2ConstPtr& cloudPtr) {
// //   // Set odometry status, add odometry factor and check if pointcloud needs to be stored
// //   _isOdomDegenerate = odomStatusPtr->degenerate;
// //   lidarOdomCallback(odomPtr);
// // }

// // int MaplabIntegrator::lidarOdomCallback(const nav_msgs::Odometry::ConstPtr& odomPtr) {
// //   if (odomPtr == nullptr) {
// //     log_info("MaplabIntegrator - nullptr passed to Odometry callback");
// //     return -1;
// //   }

// //   // Current timestamp and LiDAR pose
// //   const double ts = odomPtr->header.stamp.toSec();
// //   const auto& p = odomPtr->pose.pose;
// //   const gtsam::Pose3 T_M_L(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
// //                            gtsam::Point3(p.position.x, p.position.y, p.position.z));
// //   // IMU pose
// //   gtsam::Pose3 T_M_B = T_M_L * _T_L_B;
// //   // Compute delta in IMU frame
// //   if (_firstOdomMsg)  // Store first pose to compute zero delta incase input doesn't start from zero
// //     _lastIMUPose = T_M_B;
// //   gtsam::Pose3 T_B1_B2 = _lastIMUPose.between(T_M_B);

// //   // Add delta pose factor to graph
// //   gtsam::Key new_key;
// //   bool saveCloud = false;
// //   auto t1 = std::chrono::high_resolution_clock::now();
// //   {
// //     // If first lidar odometry message has been received
// //     std::lock_guard<std::mutex> lock(_graphMutex);
// //     if (_firstOdomMsg) {
// //       // Initialize - Add temporary identity prior factor
// //       new_key = stateKey();
// //       addPriorFactor(new_key, gtsam::Pose3::identity());
// //       saveCloud = true;
// //       // Intialize sensor transforms
// //       initSensorTransforms();
// //       _firstOdomMsg = false;
// //     } else {
// //       // Update keys
// //       auto old_key = stateKey();
// //       new_key = newStateKey();
// //       // Calculate new IMU pose estimate in G
// //       gtsam::Pose3 T_G_B = _T_G_B_inc * T_B1_B2;
// //       // Add delta and estimate as between factor
// //       addPoseBetweenFactor(old_key, new_key, T_B1_B2, T_G_B);
// //     }
// //     // Update Timestamp-Key & Key-Timestamp Map
// //     _timestampKeyMap[ts] = new_key;
// //     _keyTimestampMap[new_key] = ts;
// //     // Get updated result
// //     _T_G_B_inc = _graph->calculateEstimate<gtsam::Pose3>(X(new_key));
// //   }
// //   auto t2 = std::chrono::high_resolution_clock::now();

// //   // Save last IMU pose for calculting delta
// //   _lastIMUPose = T_M_B;

// //   // Check motion difference to know if incoming cloud should be saved
// //   gtsam::Pose3 cloud_delta = _lastCloudSavePose.between(T_M_L);
// //   if (cloud_delta.translation().norm() > _cloudSavePosDelta ||
// //       cloud_delta.rotation().rpy().norm() > _cloudSaveRotDelta) {
// //     _lastCloudSavePose = T_M_L;
// //     saveCloud = true;
// //   }

// //   // Publish Graph path
// //   // Pose Message
// //   geometry_msgs::PoseStamped poseMsg;
// //   poseMsg.header.frame_id = (_isOdomDegenerate ? "degenerate" : "");
// //   poseMsg.header.stamp = odomPtr->header.stamp;
// //   poseMsg.header.seq = new_key;
// //   createPoseMessage(_T_G_B_inc, &poseMsg);

// //   // Node Message
// //   _pathMsg.header.frame_id = _world_frame;
// //   _pathMsg.header.stamp = poseMsg.header.stamp;
// //   _pathMsg.poses.emplace_back(std::move(poseMsg));

// //   // Publish Path
// //   _pubIncrementalPath.publish(_pathMsg);

// //   // Publish Transforms
// //   publishTransforms(odomPtr->header.stamp);

// //   // DEBUG
// //   if (_verbose > 0) {
// //     log_info_COND(new_key % 10 == 0, "\033[35mODOMETRY\033[0m ts: " << odomPtr->header.stamp
// //                                                                            << ", key: " << new_key
// //                                                                            << ", T_G_B t(m): " << _T_G_B_inc.translation().transpose()
// //                                                                            << ", time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
// //   }
// //   // DEBUG

// //   // return key to associate pointcloud
// //   if (saveCloud)
// //     return new_key;
// //   else
// //     return -1;
// // }

// // void MaplabIntegrator::lidarOdomCallbackEUROC(const nav_msgs::Odometry::ConstPtr& odomPtr) {
// //   if (odomPtr == nullptr) {
// //     log_info("MaplabIntegrator - nullptr passed to Odometry callback");
// //     return;
// //   }

// //   // Current timestamp and LiDAR pose
// //   const double ts = odomPtr->header.stamp.toSec();
// //   const auto& p = odomPtr->pose.pose;
// //   const gtsam::Pose3 T_M_L(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
// //                            gtsam::Point3(p.position.x, p.position.y, p.position.z));
// //   // IMU pose
// //   gtsam::Pose3 T_M_B = T_M_L * _T_L_B;
// //   // Compute delta in IMU frame
// //   if (_firstOdomMsg)  // Store first pose to compute zero delta incase input doesn't start from zero
// //     _lastIMUPose = T_M_B;
// //   gtsam::Pose3 T_B1_B2 = _lastIMUPose.between(T_M_B);

// //   // Add delta pose factor to graph
// //   gtsam::Key new_key;
// //   bool saveCloud = false;
// //   auto t1 = std::chrono::high_resolution_clock::now();
// //   {
// //     // If first lidar odometry message has been received
// //     std::lock_guard<std::mutex> lock(_graphMutex);
// //     if (_firstOdomMsg) {
// //       // Initialize - Add temporary identity prior factor
// //       new_key = stateKey();
// //       addPriorFactor(new_key, gtsam::Pose3::identity());
// //       saveCloud = true;
// //       // Intialize sensor transforms
// //       initSensorTransforms();
// //       _firstOdomMsg = false;
// //     } else {
// //       // Update keys
// //       auto old_key = stateKey();
// //       new_key = newStateKey();
// //       // Calculate new IMU pose estimate in G
// //       gtsam::Pose3 T_G_B = _T_G_B_inc * T_B1_B2;
// //       // Add delta and estimate as between factor
// //       addPoseBetweenFactor(old_key, new_key, T_B1_B2, T_G_B);
// //     }
// //     // Update Timestamp-Key & Key-Timestamp Map
// //     _timestampKeyMap[ts] = new_key;
// //     _keyTimestampMap[new_key] = ts;
// //     // Get updated result
// //     _T_G_B_inc = _graph->calculateEstimate<gtsam::Pose3>(X(new_key));
// //   }
// //   auto t2 = std::chrono::high_resolution_clock::now();

// //   // Save last IMU pose for calculting delta
// //   _lastIMUPose = T_M_B;

// //   // Check motion difference to know if incoming cloud should be saved
// //   gtsam::Pose3 cloud_delta = _lastCloudSavePose.between(T_M_L);
// //   if (cloud_delta.translation().norm() > _cloudSavePosDelta ||
// //       cloud_delta.rotation().rpy().norm() > _cloudSaveRotDelta) {
// //     _lastCloudSavePose = T_M_L;
// //     saveCloud = true;
// //   }

// //   // Publish Graph path
// //   // Pose Message
// //   geometry_msgs::PoseStamped poseMsg;
// //   poseMsg.header.frame_id = (_isOdomDegenerate ? "degenerate" : "");
// //   poseMsg.header.stamp = odomPtr->header.stamp;
// //   poseMsg.header.seq = new_key;
// //   createPoseMessage(_T_G_B_inc, &poseMsg);

// //   // Node Message
// //   _pathMsg.header.frame_id = _world_frame;
// //   _pathMsg.header.stamp = poseMsg.header.stamp;
// //   _pathMsg.poses.emplace_back(std::move(poseMsg));

// //   // Publish Path
// //   _pubIncrementalPath.publish(_pathMsg);

// //   // Publish Transforms
// //   publishTransforms(odomPtr->header.stamp);

// //   // DEBUG
// //   if (_verbose > 0) {
// //     log_info_COND(new_key % 10 == 0, "\033[35mODOMETRY\033[0m ts: " << odomPtr->header.stamp
// //                                                                            << ", key: " << new_key
// //                                                                            << ", T_G_B t(m): " << _T_G_B_inc.translation().transpose()
// //                                                                            << ", time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
// //   }
// //   // DEBUG
// // }

// // void MaplabIntegrator::absolutePoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posePtr) {
// //   if (posePtr == nullptr || _firstOdomMsg) {
// //     log_info("MaplabIntegrator - nullptr passed to Absolute Pose Factor callback");
// //     return;
// //   }

// //   // Get T_G_C i.e. Camera(C) pose in DARPA(G)
// //   const geometry_msgs::Pose& p = posePtr->pose.pose;
// //   const gtsam::Pose3 T_G_C(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
// //                            gtsam::Point3(p.position.x, p.position.y, p.position.z));
// //   // Calculate IMU(B) pose in DARPA(G)
// //   gtsam::Pose3 T_G_B = T_G_C * _T_B_C.inverse();

// //   // Add absolute priors at FIRST key i.e. key=0
// //   auto t1 = std::chrono::high_resolution_clock::now();
// //   {
// //     std::lock_guard<std::mutex> lock(_graphMutex);
// //     // Create prior factor
// //     static auto absoluteNoise = gtsam::noiseModel::Diagonal::Sigmas(_absoluteNoise);
// //     _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), T_G_B, absoluteNoise);

// //     // If first aboslute pose - remove any previous priors at FIRST key
// //     gtsam::FactorIndices removeFactorIdx;
// //     if (_firstAbsolutePose) {
// //       removeFactorIdx.push_back(0);
// //       _firstAbsolutePose = false;
// //       // std::cout << "\033[33mABSOLUTE\033[0m First ABSOLUTE POSE received - removing old absolute pose" << std::endl;
// //     }

// //     // Update graph
// //     _graph->update(_newFactors, gtsam::Values(), removeFactorIdx);
// //     _newFactors.resize(0);

// //     // Increment total factor count
// //     incFactorCount();
// //   }
// //   auto t2 = std::chrono::high_resolution_clock::now();

// //   // Visualization markers
// //   geometry_msgs::Point mPt;
// //   mPt.x = T_G_B.translation().x();
// //   mPt.y = T_G_B.translation().y();
// //   mPt.z = T_G_B.translation().z();
// //   _absoluteMarkerMsg.points.push_back(mPt);

// //   // DEBUG
// //   if (_verbose > 0) {
// //     log_info("\033[33mABSOLUTE\033[0m"
// //                     << ", T_G_M t(m):" << _T_G_M.translation().transpose()
// //                     << ", RPY(deg): " << _T_G_M.rotation().rpy().transpose() * (180.0 / M_PI)
// //                     << ", time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
// //   }
// //   // DEBUG
// // }

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
// //         if (_verbose) log_info("\033[36mANCHOR\033[0m - Found no closest key for ts: :" << ros::Time(ts));
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
// //           if (_verbose) log_info("\033[36mANCHOR\033[0m - Found Key: " << key
// //                                                                               << ", ts: " << ros::Time(ts)
// //                                                                               << ", t(x,y,z): " << T_G_B.translation().transpose()
// //                                                                               << ", Equal: \033[32mYES\033[0m - SKIP");

// //           continue;
// //         }
// //       }

// //       // Find if Prior factor exists at Key and get Factor Index
// //       auto itr = _keyAnchorFactorIdxMap.find(key);
// //       if (itr != _keyAnchorFactorIdxMap.end()) {
// //         if (_verbose) log_info("\033[36mANCHOR\033[0m - Found Key: " << key
// //                                                                             << ", ts: " << ros::Time(ts)
// //                                                                             << ", Equal: \033[31mNO\033[0m"
// //                                                                             << ", Removing Prior with Index: " << itr->second);
// //         removeFactorIdx.push_back(itr->second);
// //       } else if (_verbose)
// //         log_info("\033[36mANCHOR\033[0m - Found Key: " << key
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
// //     if (_verbose) log_info("\033[36mANCHOR-UPDATE\033[0m - time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
// //   }
// // }

// // void MaplabIntegrator::maplabSubmapCallback(const nav_msgs::Path::ConstPtr& pathPtr) {
// //   // Check if empty message
// //   if (pathPtr == nullptr || _firstOdomMsg) {
// //     log_info("MaplabIntegrator - nullptr passed to Submap callback or graph not initialized from odometry yet");
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
// //             if (_verbose) log_info("MaplabIntegrator - Same Submap Parent/Child keys, key:" << child_key);
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
// //           if (_verbose) log_info("\033[34mSUBMAP\033[0m - " << (rmIdx == -1 ? "Added" : "Replaced") << ": P(" << parent_key << ")-C(" << child_key << "), delta_t(x,y,z):" << T_B1B2.translation().transpose());
// //         } else
// //           continue;
// //       }
// //       auto t2 = std::chrono::high_resolution_clock::now();

// //       // DEBUG
// //       if (_verbose) log_info("\033[34mSUBMAP-UPDATE\033[0m - Constraints added: " << pathPtr->poses.size() << ", time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
// //       // DEBUG
// //     } else {
// //       if (_verbose) log_info("\033[34mSUBMAP\033[0m  - Found no key for parent at ts: " << ros::Time(parent_ts) << " --- SKIPPING CHILDERN ---");
// //       return;
// //     }
// //   }
// // }

// // void MaplabIntegrator::addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose) {
// //   // Prior factor noise
// //   static auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);

// //   // Initial estimate - Use pose as estimate (For prior factors at graph init)
// //   gtsam::Values estimate;
// //   estimate.insert(X(key), pose);

// //   // Add prior factor and update graph
// //   _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), pose, priorNoise);
// //   _graph->update(_newFactors, estimate);
// //   _newFactors.resize(0);  // Reset
// //   incFactorCount();
// // }

// // void MaplabIntegrator::addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose_delta, const gtsam::Pose3& pose_estimate) {
// //   // Create Pose BetweenFactor
// //   static auto poseBFNoise = gtsam::noiseModel::Diagonal::Sigmas(_odomNoise);
// //   gtsam::BetweenFactor<gtsam::Pose3> poseBF(X(old_key), X(new_key), pose_delta, poseBFNoise);

// //   // Pose estimate
// //   gtsam::Values estimate;
// //   estimate.insert(X(new_key), pose_estimate);

// //   // Add factor and update graph
// //   _newFactors.add(poseBF);
// //   _graph->update(_newFactors, estimate);
// //   _newFactors.resize(0);  // Reset
// //   incFactorCount();
// // }

// // bool MaplabIntegrator::findClosestKeyForTS(const double ts, gtsam::Key* key) const {
// //   if (key == nullptr || _states.empty()) {
// //     return false;
// //   }
// //   static constexpr double const& eps = 0.2;  // not sure yetsrc/graph_manager.cc
// //   const double oldest_ts = (*_states.crbegin())->ts() + eps;
// //   if (ts > oldest_ts) {
// //     // The requested ts is older than the oldest one in the state vector.
// //     return false;
// //   }
// //   auto comp = [](const StatePtr& lhs, const double ts) { return lhs->ts() + eps < ts; };
// //   auto it = std::lower_bound(_states.cbegin(), _states.cend(), ts, comp);
// //   if (it == _states.cend()) {
// //     return false;
// //   }

// //   *key = (*it)->key();
// //   return true;
// // }

// // void MaplabIntegrator::updateGraphResults() {
// //   // Check if graph has initialized
// //   if (_firstOdomMsg)
// //     return;

// //   // Update results
// //   gtsam::Values result;
// //   std::unordered_map<gtsam::Key, double> keyTimestampMap;
// //   auto t_0 = std::chrono::high_resolution_clock::now();
// //   {
// //     std::lock_guard<std::mutex> lock(_graphMutex);
// //     result = _graph->calculateBestEstimate();
// //     keyTimestampMap = _keyTimestampMap;  // copy cost 36000 elements(10Hz * 1Hr) ~10ms
// //   }
// //   auto t_update = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_0).count();

// //   // Publish result at now() timestamp to account for optmization delay
// //   ros::Time resultTimestamp = ros::Time::now();

// //   // Clear constraint marker messages
// //   _anchorMarkerMsg.header.stamp = resultTimestamp;
// //   _anchorMarkerMsg.points.clear();
// //   _absoluteMarkerMsg.header.stamp = resultTimestamp;
// //   _absoluteMarkerMsg.points.clear();
// //   _submapMarkerMsg.points.clear();
// //   _submapMarkerMsg.colors.clear();
// //   _submapMarkerMsg.header.stamp = resultTimestamp;
// //   _submapParentMarkerMsg.points.clear();
// //   _submapParentMarkerMsg.colors.clear();
// //   _submapParentMarkerMsg.header.stamp = resultTimestamp;
// //   _submapTextMarkerArrayMsg.markers.clear();

// //   auto t1 = std::chrono::high_resolution_clock::now();
// //   nav_msgs::Path pathMsg;

// //   // Loop through result values - Result in absolute frame
// //   _T_G_M = result.at<gtsam::Pose3>(X(0));
// //   for (std::size_t i = 0; i < result.size(); ++i) {
// //     gtsam::Pose3 T_G_B = result.at<gtsam::Pose3>(X(i));

// //     // Create pose message.
// //     geometry_msgs::PoseStamped poseMsg;
// //     poseMsg.header.frame_id = _world_frame;
// //     poseMsg.header.seq = i;
// //     poseMsg.header.stamp = ros::Time(keyTimestampMap[i]);  // Publish each pose at timestamp corresponding to node in the graph (Note: ts=0 in case of map lookup failure)
// //     createPoseMessage(T_G_B, &poseMsg);
// //     pathMsg.poses.emplace_back(poseMsg);

// //     // Create Constraint Markers
// //     if (_pubConstraintMarkers.getNumSubscribers() > 0) {
// //       // Anchors - Check if current key has anchor attached
// //       auto anchor_itr = _keyAnchorPoseMap.find(i);
// //       if (anchor_itr != _keyAnchorPoseMap.end()) {
// //         geometry_msgs::Point p;
// //         p.x = anchor_itr->second.translation().x();
// //         p.y = anchor_itr->second.translation().y();
// //         p.z = anchor_itr->second.translation().z();
// //         _anchorMarkerMsg.points.push_back(p);
// //       }

// //       // Submap constraints - Check if current key is a parent node for a submap constraint
// //       auto parent_itr = _submapParentChildKeyMap.find(i);
// //       if (parent_itr != _submapParentChildKeyMap.end()) {
// //         // Get parent pose
// //         gtsam::Pose3 parent_pose = result.at<gtsam::Pose3>(X(i));
// //         geometry_msgs::Point parent_pt;
// //         parent_pt.x = parent_pose.translation().x();
// //         parent_pt.y = parent_pose.translation().y();
// //         parent_pt.z = parent_pose.translation().z();
// //         // Create color for constraints
// //         static int max_key = 5000;
// //         if (i > max_key)
// //           max_key = max_key * 2.0f;
// //         float val = float(i) * 255.0 / float(max_key);
// //         std_msgs::ColorRGBA color;
// //         color.r = std::round(std::sin(0.024 * val + 0) * 127 + 128) / 255.0;
// //         color.g = std::round(std::sin(0.024 * val + 2) * 127 + 128) / 255.0;
// //         color.b = std::round(std::sin(0.024 * val + 4) * 127 + 128) / 255.0;
// //         color.a = 1.0f;
// //         // Parent node sphere marker
// //         _submapParentMarkerMsg.points.push_back(parent_pt);
// //         _submapParentMarkerMsg.colors.push_back(color);
// //         // Parent node constraint number text marker
// //         if (_pubConstraintTextMarkers.getNumSubscribers() > 0) {
// //           _submapTextMarkerMsg.header.stamp = resultTimestamp;
// //           _submapTextMarkerMsg.id = i;
// //           _submapTextMarkerMsg.pose.position.x = parent_pt.x;
// //           _submapTextMarkerMsg.pose.position.y = parent_pt.y;
// //           _submapTextMarkerMsg.pose.position.z = parent_pt.z + 0.25f;
// //           _submapTextMarkerMsg.text = std::to_string(parent_itr->second.size());
// //           _submapTextMarkerArrayMsg.markers.push_back(_submapTextMarkerMsg);
// //         }
// //         // Loop through children
// //         for (const auto& childKey : parent_itr->second) {
// //           // Get Child pose
// //           gtsam::Pose3 child_pose = result.at<gtsam::Pose3>(X(childKey));
// //           geometry_msgs::Point child_pt;
// //           child_pt.x = child_pose.translation().x();
// //           child_pt.y = child_pose.translation().y();
// //           child_pt.z = child_pose.translation().z();
// //           // Add parent and child points to marker msg
// //           _submapMarkerMsg.points.push_back(parent_pt);
// //           _submapMarkerMsg.points.push_back(child_pt);
// //           _submapMarkerMsg.colors.push_back(color);
// //           _submapMarkerMsg.colors.push_back(color);
// //         }
// //       }
// //     }
// //   }

// //   // Publish Path
// //   if (_pubUpdatedPath.getNumSubscribers() > 0) {
// //     pathMsg.header.frame_id = _world_frame;
// //     pathMsg.header.stamp = resultTimestamp;
// //     _pubUpdatedPath.publish(pathMsg);
// //   }

// //   // Publish Constraint Markers
// //   if (_pubConstraintMarkers.getNumSubscribers() > 0) {
// //     if (!_anchorMarkerMsg.points.empty()) {
// //       _pubConstraintMarkers.publish(_anchorMarkerMsg);
// //     }
// //     if (!_absoluteMarkerMsg.points.empty()) {
// //       _pubConstraintMarkers.publish(_absoluteMarkerMsg);
// //     }
// //     if (!_submapMarkerMsg.points.empty()) {
// //       _pubConstraintMarkers.publish(_submapMarkerMsg);
// //     }
// //     if (!_submapParentMarkerMsg.points.empty()) {
// //       _pubConstraintMarkers.publish(_submapParentMarkerMsg);
// //     }
// //   }
// //   if (_pubConstraintTextMarkers.getNumSubscribers() > 0)
// //     _pubConstraintTextMarkers.publish(_submapTextMarkerArrayMsg);

// //   // Update last optimzed pose
// //   _T_G_B_opt = result.at<gtsam::Pose3>(X(result.size() - 1));

// //   // DEBUG
// //   auto t2 = std::chrono::high_resolution_clock::now();
// //   if (_verbose) {
// //     log_info("\033[36mGRAPH UPDATE\033[0m - time(us):" << t_update << ", Map Build(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
// //     log_info("\033[36mGRAPH UPDATE\033[0m - number of factors: " << _factor_count);
// //   }
// //   // DEBUG
// // }

// // void MaplabIntegrator::publishTransforms(const ros::Time& ts) {
// //   // Absolute(DARPA)-to-RobotMap
// //   if (_publishWorldtoMapTf) {
// //     geometry_msgs::TransformStamped T_G_M;
// //     T_G_M.header.stamp = ts;
// //     T_G_M.header.frame_id = _world_frame;
// //     T_G_M.child_frame_id = _map_frame;
// //     T_G_M.transform.rotation.x = _T_G_M.rotation().toQuaternion().x();
// //     T_G_M.transform.rotation.y = _T_G_M.rotation().toQuaternion().y();
// //     T_G_M.transform.rotation.z = _T_G_M.rotation().toQuaternion().z();
// //     T_G_M.transform.rotation.w = _T_G_M.rotation().toQuaternion().w();
// //     T_G_M.transform.translation.x = _T_G_M.translation().x();
// //     T_G_M.transform.translation.y = _T_G_M.translation().y();
// //     T_G_M.transform.translation.z = _T_G_M.translation().z();
// //     _tb.sendTransform(T_G_M);
// //   }

// //   // Base(IMU)-to-Absolute(DARPA) - Incremental Update
// //   geometry_msgs::TransformStamped T_G_B_inc;
// //   T_G_B_inc.header.stamp = ts;
// //   T_G_B_inc.header.frame_id = _world_frame;
// //   T_G_B_inc.child_frame_id = _base_frame + "_increment";
// //   T_G_B_inc.transform.rotation.x = _T_G_B_inc.rotation().toQuaternion().x();
// //   T_G_B_inc.transform.rotation.y = _T_G_B_inc.rotation().toQuaternion().y();
// //   T_G_B_inc.transform.rotation.z = _T_G_B_inc.rotation().toQuaternion().z();
// //   T_G_B_inc.transform.rotation.w = _T_G_B_inc.rotation().toQuaternion().w();
// //   T_G_B_inc.transform.translation.x = _T_G_B_inc.translation().x();
// //   T_G_B_inc.transform.translation.y = _T_G_B_inc.translation().y();
// //   T_G_B_inc.transform.translation.z = _T_G_B_inc.translation().z();
// //   _tb.sendTransform(T_G_B_inc);
// //   // PoseStamped message
// //   if (_pubIncPoseStamped.getNumSubscribers() > 0) {
// //     geometry_msgs::PoseStamped T_G_B_inc_poseMsg;
// //     convertTransfromToPose(T_G_B_inc, T_G_B_inc_poseMsg);
// //     _pubIncPoseStamped.publish(T_G_B_inc_poseMsg);
// //   }

// //   // Base(IMU)-to-Absolute(DARPA) - Optimized
// //   geometry_msgs::TransformStamped T_G_B_opt;
// //   T_G_B_opt.header.stamp = ts;
// //   T_G_B_opt.header.frame_id = _world_frame;
// //   T_G_B_opt.child_frame_id = _base_frame + "_optimized";
// //   T_G_B_opt.transform.rotation.x = _T_G_B_opt.rotation().toQuaternion().x();
// //   T_G_B_opt.transform.rotation.y = _T_G_B_opt.rotation().toQuaternion().y();
// //   T_G_B_opt.transform.rotation.z = _T_G_B_opt.rotation().toQuaternion().z();
// //   T_G_B_opt.transform.rotation.w = _T_G_B_opt.rotation().toQuaternion().w();
// //   T_G_B_opt.transform.translation.x = _T_G_B_opt.translation().x();
// //   T_G_B_opt.transform.translation.y = _T_G_B_opt.translation().y();
// //   T_G_B_opt.transform.translation.z = _T_G_B_opt.translation().z();
// //   _tb.sendTransform(T_G_B_opt);
// //   // PoseStamped message
// //   if (_pubOptPoseStamped.getNumSubscribers() > 0) {
// //     geometry_msgs::PoseStamped T_G_B_opt_poseMsg;
// //     convertTransfromToPose(T_G_B_opt, T_G_B_opt_poseMsg);
// //     _pubOptPoseStamped.publish(T_G_B_opt_poseMsg);
// //   }
// // }

// // bool MaplabIntegrator::createPoseMessage(const gtsam::Pose3& pose, geometry_msgs::PoseStamped* poseMsg) const {
// //   if (poseMsg == nullptr) {
// //     return false;
// //   }
// //   poseMsg->pose.position.x = pose.translation().x();
// //   poseMsg->pose.position.y = pose.translation().y();
// //   poseMsg->pose.position.z = pose.translation().z();
// //   poseMsg->pose.orientation.x = pose.rotation().toQuaternion().x();
// //   poseMsg->pose.orientation.y = pose.rotation().toQuaternion().y();
// //   poseMsg->pose.orientation.z = pose.rotation().toQuaternion().z();
// //   poseMsg->pose.orientation.w = pose.rotation().toQuaternion().w();

// //   return true;
// // }

// // void MaplabIntegrator::convertTransfromToPose(const geometry_msgs::TransformStamped& t, geometry_msgs::PoseStamped& p) {
// //   // Header
// //   p.header = t.header;
// //   // Translation
// //   p.pose.position.x = t.transform.translation.x;
// //   p.pose.position.y = t.transform.translation.y;
// //   p.pose.position.z = t.transform.translation.z;
// //   // Rotation
// //   p.pose.orientation = t.transform.rotation;
// // }

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
// //     log_info("MaplabIntegrator - Submaps connected by mutliple between factors");
// //     std::cout << "Indices found: ";
// //     for (auto& e : result)
// //       std::cout << e << " ";
// //     std::cout << ", between keys: " << parent_key << "/" << child_key << std::endl;
// //   }

// //   return output;
// // }

// // void MaplabIntegrator::initSensorTransforms() {
// //   // Setup IMU-to-LiDAR transforms
// //   _T_L_B = gtsam::Pose3(gtsam::Rot3(q.toRotationMatrix()), t);
// //   std::cout << "\033[36mMaplabIntegrator\033[0m - T_imu2lidar(T_LB):\n"
// //             << _T_L_B.matrix() << std::endl;

// //   // Setup IMU-to-Camera transforms
// //   _T_B_C = gtsam::Pose3(gtsam::Rot3(q.toRotationMatrix()), t);
// //   std::cout << "\033[36mMaplabIntegrator\033[0m - T_cam2imu(T_BC):\n"
// //             << _T_B_C.matrix() << std::endl;
// // }

}  // namespace fgsp
