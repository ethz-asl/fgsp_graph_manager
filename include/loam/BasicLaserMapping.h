#ifndef BASIC_LASER_MAPPING_H
#define BASIC_LASER_MAPPING_H
// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "CircularBuffer.h"
#include "Twist.h"
#include "loam/OptStatus.h"
#include "math_utils.h"
#include "time_utils.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//CUSTOMIZATION
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <unordered_set>
#include <gtsam/geometry/Pose3.h>
//CUSTOMIZATION

namespace loam {

/** IMU state data. */
typedef struct IMUState2 {
  /** The time of the measurement leading to this state (in seconds). */
  Time stamp;

  /** The current roll angle. */
  Angle roll;

  /** The current pitch angle. */
  Angle pitch;

  /** \brief Interpolate between two IMU states.
   *
   * @param start the first IMU state
   * @param end the second IMU state
   * @param ratio the interpolation ratio
   * @param result the target IMU state for storing the interpolation result
   */
  static void interpolate(const IMUState2& start,
                          const IMUState2& end,
                          const float& ratio,
                          IMUState2& result) {
    float invRatio = 1 - ratio;

    result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
    result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
  };
} IMUState2;

class BasicLaserMapping {
 public:
  explicit BasicLaserMapping(const float& scanPeriod = 0.1, const size_t& maxIterations = 10);

  /** \brief Try to process buffered data. */
  bool process(Time const& laserOdometryTime);
  void updateIMU(IMUState2 const& newState);
  void updateOdometry(double pitch, double yaw, double roll, double x, double y, double z);
  void updateOdometry(Twist const& twist);

  auto& laserCloud() { return *_laserCloudFullRes; }
  auto& laserCloudCornerLast() { return *_laserCloudCornerLast; }
  auto& laserCloudSurfLast() { return *_laserCloudSurfLast; }

  void setScanPeriod(float val) { _scanPeriod = val; }
  void setMaxIterations(size_t val) { _maxIterations = val; }
  void setMinIterations(size_t val) { _minIterations = val; }
  void setDeltaTAbort(float val) { _deltaTAbort = val; }
  void setDeltaRAbort(float val) { _deltaRAbort = val; }
  void useExtRotationAlignment(bool b) { _useExtRotationAlignment = b; }

  auto& downSizeFilterCorner() { return _downSizeFilterCorner; }
  auto& downSizeFilterSurf() { return _downSizeFilterSurf; }
  auto& downSizeFilterMapVis() { return _downSizeFilterMapVis; }

  auto frameCount() const { return _frameCount; }
  auto scanPeriod() const { return _scanPeriod; }
  auto maxIterations() const { return _maxIterations; }
  auto minIterations() const { return _minIterations; }
  auto deltaTAbort() const { return _deltaTAbort; }
  auto deltaRAbort() const { return _deltaRAbort; }

  auto const& transformAftMapped() const { return _transformAftMapped; }
  auto const& transformBefMapped() const { return _transformBefMapped; }
  auto const& laserCloudSurroundDS() const { return *_laserCloudSurroundDS; }

  bool createDownsizedMap();
  void transformFullResToMap();
  bool hasFreshMap() const { return _downsizedMapCreated; }

  //CUTOMIZATION
  /** \brief Resets Map Parameters and Containers */
  void resetMapParamsAndContainers();
  /** \brief Sets degeneracy status of laser odometry
   *
   * @param laser odometry status
   */
  void setlaserOdometryStatus(const bool currentStatus) { _isLaserOdometryDegenerate = currentStatus; }

  /** \brief Update timestamp corresponding to latest PCL message.
   *
   * @param ROS timestamp of latest PCL message
   */
  void updatePCLTime(const ros::Time& currentTimeStamp);

  /** \brief Set Eigen Value threshold for degeneracy
   *
   * @param val: threshold value for eigen value
   */
  void setDegenEigenThreshold(float val) { _mapDegenEigVal = val; }

  /** \brief Set External Odometry Prior max norm threshold
   *
   * @param val: threshold value for max external odometry position prior norm
   */
  void setExtOdomPriorNormThreshold(double val) { _mapPriorNormThresh = val; }

  /** \brief Calculate external prior in LOAM(camera_init) coordinates as return as translation and rotation(euler) vectors in [x,y,z] order
   * @param transPriorXYZ: calculated translation prior
   * @param rotPriorXYZ: calculated rotation prior
   * @param useFallback: use fallback for prior calculation
   * @return bool: return true if frame look-up and prior calculation is successful
   */
  bool calcExternalPrior(Eigen::Vector3d& transPriorXYZ, Eigen::Vector3d& rotPriorXYZ, bool useFallback = false);

  // Submap Co-Localization
  void setSavedSubmapUse(bool useSubmap) { _useSavedSubmapsForInit = useSubmap; }
  void setSubmapLocalizationGuess(std::vector<float>& guess) { _W_submapLocGuess = guess; }
  void setSavedSubmapFilepath(std::string& path) { _saved_submaps_filepath = path; }
  auto getSavedSubmapFilepath() { return _saved_submaps_filepath; }
  auto& getSavedCornerSubmap() { return *_savedCornerSubmap; }
  auto& getSavedSurfSubmap() { return *_savedSurfSubmap; }
  auto& getCurrentCornerSubmap() { return *_laserCloudCornerArray[_laserCloudNum / 2]; }
  auto& getCurrentSurfSubmap() { return *_laserCloudSurfArray[_laserCloudNum / 2]; }

  pcl::PointCloud<pcl::PointXYZI>& laserMap();
  pcl::PointCloud<pcl::PointXYZI>& getSubmapSearchCloud();

  // Set internal map parameters
  void setMapCubeSize(double s) { _mapCubeSize = s; }
  void setMapWidthInCubes(size_t w) { _laserCloudWidth = w; }
  void setMapHeightInCubes(size_t h) { _laserCloudHeight = h; }
  void setMapDepthInCubes(size_t d) { _laserCloudDepth = d; }
  const size_t getMapWidthInCubes() { return _laserCloudWidth; }
  const size_t getMapHeightInCubes() { return _laserCloudHeight; }
  const size_t getMapDepthInCubes() { return _laserCloudDepth; }
  void setMapStartLocWidthInCubes(int w) { _laserCloudCenWidth = w; }
  void setMapStartLocHeightInCubes(int h) { _laserCloudCenHeight = h; }
  void setMapStartLocDepthInCubes(int d) { _laserCloudCenDepth = d; }
  void setNumNeighborSubmapCubes(int n) { _neighborSubmapCubes = n; }
  // Set frames and parameters for integration of external motion priors
  void setExternalPriorStatus(bool b) { _extPriorAvailable = b; }
  void setExternalOdometryFrame(const std::string& s) { _extOdomFrame = s; }
  void setExternalFixedFrame(const std::string& s) { _extFixedFrame = s; }
  void setExternalSensorFrame(const std::string& s) { _extSensorFrame = s; }
  void setExternalOdometryTimeOffset(const double d) { _extOdomTimeOffset = d; }
  void setFallbackExternalPriorStatus(bool b) { _fallbackExtPriorAvailable = b; }
  void setFallbackExternalOdometryFrame(const std::string& s) { _fallbackExtOdomFrame = s; }
  void setFallbackExternalFixedFrame(const std::string& s) { _fallbackExtFixedFrame = s; }
  void setFallbackExternalSensorFrame(const std::string& s) { _fallbackExtSensorFrame = s; }
  void setFallbackExternalOdomTimeOffset(const double d) { _fallbackExtOdomTimeOffset = d; }
  void setLidarFrame(const std::string& s) { _lidarFrame = s; }
  void forceExternalPriorUseForMapping(bool b) { _forceExternalPriorUseForMapping = b; }
  void setInputPCLRotationStatus(bool b) { _isInputPCLRotated = b; }
  void setLiDARtoInputPCLRotation(const tf::Transform t) { _Rot_inCloudLiDAR = t; }
  loam::OptStatus _optStatusMsg;

  //DEBUG
  void setVerboseLevel(int verbose) { _verboseLevel = verbose; }
  //CUSTOMIZATION

 private:
  //Functions
  /** Run an optimization. */
  void optimizeTransformTobeMapped();

  void transformAssociateToMap();
  void transformUpdate();
  void pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
  void pointAssociateTobeMapped(const pcl::PointXYZI& pi, pcl::PointXYZI& po);

  size_t toIndex(int i, int j, int k) const { return i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k; }

  //Members
  Time _laserOdometryTime;

  float _scanPeriod;  ///< time per scan
  const int _stackFrameNum;
  const int _mapFrameNum;
  long _frameCount;
  long _mapFrameCount;

  size_t _maxIterations;  ///< maximum number of iterations
  size_t _minIterations;  ///< minimum number of iterations
  float _deltaTAbort;     ///< optimization abort threshold for deltaT
  float _deltaRAbort;     ///< optimization abort threshold for deltaR

  int _laserCloudCenWidth;  //smk: Center/Start Location in Map Cube Coordinates 10x10x5
  int _laserCloudCenHeight;
  int _laserCloudCenDepth;

  size_t _laserCloudWidth;  //smk: Map size in Number of Cubes 21x21x11
  size_t _laserCloudHeight;
  size_t _laserCloudDepth;
  size_t _laserCloudNum;

  double _mapCubeSize = 50.0;                   //CUSTOMIZATION: Size of Map Cube in meters
  int _neighborSubmapCubes = 2;                 //CUSTOMIZATION: Number of Neighboring cubes considered in +/- direction for creation of submap for matching
  std::unordered_set<size_t> _occupiedCubeInd;  //CUSTOMIZATION: Indices of occupied map cubes

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerLast;  ///< last corner points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfLast;    ///< last surface points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudFullRes;     ///< last full resolution cloud

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStack;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStack;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStackDS;  ///< down sampled
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStackDS;    ///< down sampled

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurround;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurroundDS;  ///< down sampled
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerFromMap;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfFromMap;

  pcl::PointCloud<pcl::PointXYZI> _laserCloudOri;
  pcl::PointCloud<pcl::PointXYZI> _coeffSel;

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerArray;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfArray;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerDSArray;  ///< down sampled
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfDSArray;    ///< down sampled

  std::vector<size_t> _laserCloudValidInd;
  std::vector<size_t> _laserCloudSurroundInd;

  Twist _transformSum, _transformIncre, _transformTobeMapped, _transformBefMapped, _transformAftMapped;
  CircularBuffer<IMUState2> _imuHistory;  ///< history of IMU states

  pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterCorner;  ///< voxel filter for down sizing corner clouds
  pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterSurf;    ///< voxel filter for down sizing surface clouds
  pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterMapVis;  ///< voxel filter for down sizing map clouds for visualization

  bool _downsizedMapCreated = false;

  //CUSTOMIZATION
  std::ofstream _mappingLogFile;            // logging file for debugging
  float _mapDegenEigVal;                    // optimization degeneracy checking eigen value threshold
  double _mapPriorNormThresh;               // external odometry prior norm threshold(e.g. allowed max velocity is 1m/s so at 5Hz of LaserMapping max distance can be 0.2m (0.25m with some tolerance))
  bool _isLaserOdometryDegenerate = false;  // flag to check if laser odometry is degenrate
  ros::Time _currentPCLTime, _lastPCLTime;  // timestamps of last and current pointclouds

  //Submap Co-Localization
  pcl::PointCloud<pcl::PointXYZI>::Ptr _savedCornerSubmap, _savedSurfSubmap;
  std::string _saved_submaps_filepath;
  bool _useSavedSubmapsForInit = false;  //use external saved submaps for initialization
  bool _IncreOptIter = false;
  std::vector<float> _W_submapLocGuess{0, 0, 0, 0, 0, 0};  //x,y,z,yaw,pitch,roll //initial guess for re-localization in submap expressed in ROS coordinates

  //External Motion Prior Parameters
  tf::TransformListener _tfListener;              // Transform Listener for calculating external estimate
  bool _isExternalPriorHealthy = false;           // Flag to check if external prior is of good quality
  bool _extPriorAvailable = false;                // Flag to check if 'Primary' external prior is available
  std::string _extOdomFrame = "";                 // External Prior odometry frame name
  std::string _extFixedFrame = "";                // External Prior fixed frame name
  std::string _extSensorFrame = "";               // External Prior sensor frame name
  double _extOdomTimeOffset = 0.0;                // Timeoffset between LiDAR pointcloud and external source
  bool _fallbackExtPriorAvailable = false;        // Flag to check if 'Fallback' external prior is available
  std::string _fallbackExtOdomFrame = "";         // Fallback External Prior odometry frame name
  std::string _fallbackExtFixedFrame = "";        // Fallback External Prior fixed frame name
  std::string _fallbackExtSensorFrame = "";       // Fallback External Prior sensor frame name
  double _fallbackExtOdomTimeOffset = 0.0;        // Timeoffset between LiDAR pointcloud and Fallbackexternal source
  std::string _lidarFrame = "";                   // LiDAR frame name - used to lookup LiDAR-to-ExternalSensor Frame
  bool _useExtRotationAlignment = false;          // Use external frame for gravity alignment (use a prior)
  Twist _externalEstimate;                        // Calcualted External Estimate
  bool _forceExternalPriorUseForMapping = false;  // Flag to force use of FULL external prior instead of LaserOdometry TRANSLATION Only Estimate
  bool _isInputPCLRotated = false;
  tf::Transform _Rot_inCloudLiDAR;

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserMap;           // Full Map for visualization - Downsampled
  pcl::PointCloud<pcl::PointXYZI>::Ptr _submapSearchCloud;  // Cloud to searched within current submap

  //DEBUG
  int _verboseLevel = 0;
  //CUSTOMIZATION
};

}  // end namespace loam
#endif