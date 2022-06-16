#pragma once

#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <vector>

// #include <message_filters/subscriber.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/sync_policies/exact_time.h>
// #include <message_filters/synchronizer.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// #include "visualization_msgs/Marker.h"
// #include "visualization_msgs/MarkerArray.h"

#define SLOW_BUT_CORRECT_BETWEENFACTOR  // increases accuracy in handling rotations
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// #include "loam/GraphState.hpp"
// #include "loam/OptStatus.h"
#include "graph_manager/graph_manager_config.h"

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)

namespace fgsp {

class GraphManager {
 public:
  GraphManager(GraphManagerConfig const& config);
  // Setup
  //  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  //  private:
  //   //Callbacks
  //   void syncCallbackHandler(const nav_msgs::OdometryConstPtr& odomPtr, const OptStatusConstPtr& odomStatusPtr, const sensor_msgs::PointCloud2ConstPtr& cloudPtr);
  //   int lidarOdomCallback(const nav_msgs::Odometry::ConstPtr& odomPtr);
  void odometryCallback(nav_msgs::msg::Odometry const& odom);
  //   void cloudCallback(int key, const sensor_msgs::PointCloud2ConstPtr& cloudPtr);
  //   void absolutePoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posePtr);
  //   void maplabAnchorCallback(const nav_msgs::Path::ConstPtr& pathPtr);
  //   void maplabSubmapCallback(const nav_msgs::Path::ConstPtr& pathPtr);

  // Get State key
  auto stateKey() const -> gtsam::Key { return state_key_; }
  // State Key increment
  auto newStateKey() -> gtsam::Key { return ++state_key_; }
  // Add prior
  void addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose);
  // Add a pose between factor
  void addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose_delta, const gtsam::Pose3& pose_estimate);
  //   //Update ISAM graph
  void updateGraphResults();
  //   //Publish associated transforms
  //   void publishTransforms(const ros::Time& ts);

  //   //Utility methods
  bool createPoseMessage(const gtsam::Pose3& pose, geometry_msgs::msg::PoseStamped* pose_msg) const;

  //   //Lookup maps for key-factor association
  //   void updateKeyAnchorFactorIdxMap(const gtsam::Key key) { _keyAnchorFactorIdxMap[key] = factor_count_ - 1; }
  //   void updateKeyAnchorPoseMap(const gtsam::Key key, const gtsam::Pose3& pose) { _keyAnchorPoseMap[key] = pose; }
  //   void updateKeySubmapFactorIdxMap(const gtsam::Key parent_key, const gtsam::Key child_key);
  //   int findSubmapFactorIdx(const gtsam::Key parent_key, const gtsam::Key child_key, bool erase = false);

  //   //Factor count increment and retreivel
  void incFactorCount() { ++factor_count_; }
  auto getFactorCount() -> std::size_t { return factor_count_; }

  //   //Intialize Sensor extrinsic transforms
  //   void initSensorTransforms();

  //   //Members

  //   //Subscribers
  //   ros::Subscriber _subOdomEUROC;     // Odometry subscriber for odometry constraints
  //   ros::Subscriber _subAbsolutePose;  // Absolute pose constraints from Apriltags
  //   ros::Subscriber _subMaplabAnchor;  // Maplab published Anchor constraints
  //   ros::Subscriber _subMaplabSubmap;  // Maplab published Submap-to-Submap contraints

  //   // //Synced subscriber
  //   message_filters::Subscriber<nav_msgs::Odometry> _subOdom;                                                                //Odometry subscriber for odometry constraints
  //   message_filters::Subscriber<OptStatus> _subOdomStatus;                                                                   //Odometry status subscriber for marking variables at which odometry is degenerate
  //   message_filters::Subscriber<sensor_msgs::PointCloud2> _subCloud;                                                         //Pointcloud subscriber for saving clouds for building a map
  //   typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, OptStatus, sensor_msgs::PointCloud2> _syncPolicy;  // ROS Sync Policy Object
  //   boost::shared_ptr<message_filters::Synchronizer<_syncPolicy>> _syncPtr;                                                  // ROS  Sync Policy Message Filter

  //   //Publishers
  //   ros::Publisher _pubIncrementalPath, _pubIncPoseStamped;  // Current (Incremental) state of graph, path and posestamped
  //   ros::Publisher _pubUpdatedPath, _pubOptPoseStamped;      // Optimized graph state, path and posestamped
  //   nav_msgs::Path _pathMsg;                                 // ROS path message for graph
  //   ros::Publisher _pubMap;                                  // Publish corrected Map

  //   //Frame names and transforms
  //   bool _publishWorldtoMapTf = false;
  //   std::string _world_frame = "world";
  //   std::string _map_frame = "map";
  //   std::string _base_frame = "imu";
  //   tf::TransformBroadcaster _tb;              // transform broadcaster
  //   geometry_msgs::TransformStamped _optPose;  // Optimized Pose
  //   geometry_msgs::TransformStamped _incPose;  // incremental Pose

  //   //Transforms
  //   tf::TransformListener _tl;       // Transform Listener for calculating external estimate
  //   std::string _lidar_frame = "";   //LiDAR frame name - used for LiDAR-to-Sensor transform lookup
  // std::string _camera_frame = "";  //Frame of camera used for apriltag detection (absolute poses)
  // std::string _imu_frame = "";     //Frame of IMU used by maplab
  gtsam::Pose3 T_O_B_;      // IMU(B) to LiDAR(L)
  gtsam::Pose3 T_B_C_;      // camera(C) to  IMU(B)
  gtsam::Pose3 T_G_M_;      // Robot(Local) Map(M) to DARPA(G) - Local Robot Map start at origin MUST BE SET TO ZERO ON INIT in (B) frame - this needs to be updated to make the local graph expressed w.r.t (G)
  gtsam::Pose3 T_G_B_opt_;  // IMU to DARPA(G) - optimized
  gtsam::Pose3 T_G_B_inc_;  // IMU to DARPA(G) - incremental

  // Factor graph
  std::size_t factor_count_ = 0;            // Counter for Total factors (existing + removed)
  std::mutex graphMutex_;                   // For adding new factors and graph update
  gtsam::ISAM2Params params_;               // Graph parameters
  gtsam::NonlinearFactorGraph newFactors_;  // New factors to be added to the graph
  std::shared_ptr<gtsam::ISAM2> graph_;     // iSAM2 GRAPH object
  gtsam::Key state_key_ = 0;                // Current state key
                                            // std::vector<StatePtr> _states;  // Vecotr of states //TODO deprecate

  // Factor noise vectors - ORDER RPY(rad) - XYZ(meters)
  gtsam::Vector6 odomNoise_;      // Odometry BetweenFactor Noise
  gtsam::Vector6 absoluteNoise_;  // Absolute(AprilTag) PriorFactor Noise
  gtsam::Vector6 submapNoise_;    // Submap BetweenFactor Noise
  gtsam::Vector6 anchorNoise_;    // Anchor PriorFactor Noise

  // Odometry factor
  bool first_odom_msg_ = true;
  gtsam::Pose3 last_IMU_pose_;

  // Absolute pose factor
  bool firstAbsolutePose_ = true;

  // Lookup map objects for key-to-factorIndex associations
  std::unordered_map<double, gtsam::Key> timestampKeyMap_;                        // Timestamp-Key map for lookup of keys corresponding to odometry timestamps
  std::unordered_map<gtsam::Key, double> keyTimestampMap_;                        // Key-Timestamp map used for publishing graph node timestamps for path message publishing
  std::unordered_map<gtsam::Key, size_t> keyAnchorFactorIdxMap_;                  // Key-PriorFactorIndex map for lookup of indices of prior factor add at key for Anchor poses
  std::unordered_map<gtsam::Key, gtsam::Pose3> keyAnchorPoseMap_;                 // Key-AnchorPose map for lookup of applied anchor pose as prior factor at Key
  std::unordered_map<gtsam::Key, std::set<size_t>> keySubmapFactorIdxMap_;        // Key-SubmapBetweenFactorIndex map for lookup of indices of betweenfactor added at key for Submap constraints
  std::unordered_map<gtsam::Key, std::set<gtsam::Key>> submapParentChildKeyMap_;  // Parent-Child keys for visualization of relative submap constrinats

  //   //Timer-based map/result update and publish
  //   double _updateResultsInterval = 30.0;  //Interval (seconds) for graph update callback
  //   ros::Timer _timerUpdateResults;        //Timer callback for graph update

  //   //Pointcloud
  gtsam::Pose3 lastCloudSavePose_;  // pose of last pointcloud saved

  //   //Debug
  // int _verbose = 0;                                           //Verbosity level of MaplabIntegrator (0:quiet)
  //   ros::Publisher _pubConstraintMarkers;                       //Constraint marker publisher
  //   visualization_msgs::Marker _absoluteMarkerMsg;              //Absolute contraint marker messages
  //   visualization_msgs::Marker _anchorMarkerMsg;                //Anchor(Unary) contraint marker messages
  //   visualization_msgs::Marker _submapMarkerMsg;                //Submap(Relative) contraint marker messages
  //   visualization_msgs::Marker _submapParentMarkerMsg;          //Submap-Parent node contraint marker messages
  //   ros::Publisher _pubConstraintTextMarkers;                   //Constraint text marker publisher
  //   visualization_msgs::Marker _submapTextMarkerMsg;            //Constraint text marker message
  //   visualization_msgs::MarkerArray _submapTextMarkerArrayMsg;  //Constraint text marker message array
  GraphManagerConfig const& config_;
};

}  // namespace fgsp
