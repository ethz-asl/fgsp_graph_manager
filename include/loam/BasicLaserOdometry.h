#ifndef BASIC_LASER_ODOMETRY_H
#define BASIC_LASER_ODOMETRY_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Twist.h"
#include "math_utils.h"
#include "nanoflann_pcl.h"

//CUSTOMIZATION
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <fstream>
#include "CircularBuffer.h"
#include "GraphManager.hpp"
#include "ImuManager.hpp"
#include "loam/OptStatus.h"
//CUSTOMIZATION

namespace loam {

/** \brief Implementation of the LOAM laser odometry component.
 *
 */
class BasicLaserOdometry {
 public:
  explicit BasicLaserOdometry(float scanPeriod = 0.1, size_t maxIterations = 25);

  /** \brief Try to process buffered data. */
  void process();
  void updateIMU(pcl::PointCloud<pcl::PointXYZ> const& imuTrans);

  auto& cornerPointsSharp() { return _cornerPointsSharp; }
  auto& cornerPointsLessSharp() { return _cornerPointsLessSharp; }
  auto& surfPointsFlat() { return _surfPointsFlat; }
  auto& surfPointsLessFlat() { return _surfPointsLessFlat; }
  auto& laserCloud() { return _laserCloud; }

  auto const& transformSum() { return _transformSum; }
  auto const& transform() { return _transform; }
  auto const& lastCornerCloud() { return _lastCornerCloud; }
  auto const& lastSurfaceCloud() { return _lastSurfaceCloud; }

  void setScanPeriod(float val) { _scanPeriod = val; }
  void setMaxIterations(size_t val) { _maxIterations = val; }
  void setMinIterations(size_t val) { _minIterations = val; }
  void setDeltaTAbort(float val) { _deltaTAbort = val; }
  void setDeltaRAbort(float val) { _deltaRAbort = val; }

  auto frameCount() const { return _frameCount; }
  auto scanPeriod() const { return _scanPeriod; }
  auto maxIterations() const { return _maxIterations; }
  auto minIterations() const { return _minIterations; }
  auto deltaTAbort() const { return _deltaTAbort; }
  auto deltaRAbort() const { return _deltaRAbort; }

  auto const& graphIMUBias() const { return _imuBiasMsg; }

  /** \brief Transform the given point cloud to the end of the sweep.
   *
   * @param cloud the point cloud to transform
   */
  size_t transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  //CUTOMIZATION
  /** \brief Update timestamp corresponding to latest PCL message.
   *
   * @param ROS timestamp of latest PCL message
   */
  void updatePCLTime(const ros::Time& currentTimeStamp);

  /** \brief Find nearest external odometry message to given timestamp
   *
   * @param timestamp: ROS timestamp to lookup
   * @param OdomMsgROS: returned nearest external odometry message
   * @return true if a message with timestamp difference less than threshold is found
   *
   */
  // bool getNearestOdomMsgs(const ros::Time& timestamp, nav_msgs::Odometry& OdomMsg);

  /** \brief Set Eigen Value threshold for degeneracy
   *
   * @param val: threshold value for eigen value
   */
  void setDegenEigenThreshold(float val) { _odomDegenEigVal = val; }

  /** \brief Set External Odometry Relative Max Translation threshold
   *
   * @param val: threshold value for max external odometry position prior norm
   */
  void setExtOdomRelativeTranslationMax(double val) { _odomRelativeTranslationMax = val; }

  /** \brief Set External Odometry Relative Max Rotation threshold
   *
   * @param val: threshold value for max external odometry rotation prior norm
   */
  void setExtOdomRelativeRotationMax(double val) { _odomRelativeRotationMax = val; }

  /** \brief Return if Laser odometry has been degenerate in past 5 messages
   */
  bool getLaserOdometryStatus() const { return _wasDegenerate; }

  /** \brief Return if Laser odometry has been degenerate in past 5 messages
   * @param lastOdomMsg, currOdomMsg: odometry messages corresponding to last and current pointcloud
   * @param thresh: threshold for safety metric
   * @return true if external odometry is healthy
   */
  bool checkOdometryHealth(const nav_msgs::Odometry& lastMsg, const nav_msgs::Odometry& currMsg, const double& thresh);

  /** \brief Calculate Doptimality metric for checking external odometry
   * @param covMat: covariance matrix of external odometry
   * @return dOptVal: return calculated D-optimalitry metric
   */
  double calcDoptimiality(const Eigen::MatrixXd& covMat) { return std::exp(std::log(std::pow(covMat.determinant(), (1.0 / covMat.rows())))); }

  /** \brief Calculate Aoptimality metric for checking external odometry
   * @param covMat: covariance matrix of
   * @return aOptVal: return calculated A-optimalitry metric
   */
  double calcAoptimiality(const Eigen::MatrixXd& covMat) { return covMat.trace(); }

  /** \brief Calculate Entropy/Differential Entropy metric
   * @param covMat: covariance matrix of
   * @return entropy: return calculated entropy metric
   */
  double calcDifferentialEntropy(const Eigen::MatrixXd& covMat) { return (0.5 * std::log(std::pow((2.0 * M_PI * std::exp(1)), covMat.rows()) * covMat.determinant())); }

  /** \brief Calculate external prior in LOAM(camera_init) coordinates as return as translation and rotation(euler) vectors in [x,y,z] order
   * @param transPriorXYZ: calculated translation prior
   * @param rotPriorXYZ: calculated rotation prior
   * @param useFallback: use fallback for prior calculation
   * @return bool: return true if frame look-up and prior calculation is successful
   */
  bool calcExternalPrior(Eigen::Vector3d& transPriorXYZ, Eigen::Vector3d& rotPriorXYZ, bool useFallback = false);

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
  void setInputPCLRotationStatus(bool b) { _isInputPCLRotated = b; }
  void setLiDARtoInputPCLRotation(const tf::Transform t) { _Rot_inCloudLiDAR = t; }
  bool gravityAttitudeInitialized() { return _gravityAttitudeInit; }
  auto const& getLiDARGravityInitAttitude() { return _gravityAttitude; }
  void setUndistortInputCloud(bool b) { _undistortInputCloud = b; }
  void setImuFrame(const std::string& s) { _imuFrame = s; }
  void setImuTimeOffset(const double d) { _imuTimeOffset = d; }
  void setZeroMotionDetection(const bool b) { _zeroMotionDetection = b; }
  void setUseGravityRollPitchFactors(const bool b) { _gravityRollPitchFactors = b; }
  void setUseIMURollPitchForGraphInit(const bool b) { _initGraphRollPicthFromIMU = b; }

  void setVerboseLevel(int verbose) { _verboseLevel = verbose; }
  loam::OptStatus _optStatusMsg;
  //CUTOMIZATION

 protected:
  /** \brief IMU buffer*/
  ImuManager _imuBuffer;
  /** \brief Factor graph*/
  GraphManager _graphMgr;

 private:
  /** \brief Transform the given point to the start of the sweep.
   *
   * @param pi the point to transform
   * @param po the point instance for storing the result
   */
  void transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po);

  void pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                         const Angle& blx, const Angle& bly, const Angle& blz,
                         const Angle& alx, const Angle& aly, const Angle& alz,
                         Angle& acx, Angle& acy, Angle& acz);

  void accumulateRotation(Angle cx, Angle cy, Angle cz,
                          Angle lx, Angle ly, Angle lz,
                          Angle& ox, Angle& oy, Angle& oz);

  bool updateFactorGraph();

  //Member variables
  float _scanPeriod;      ///< time per scan
  long _frameCount;       ///< number of processed frames
  size_t _maxIterations;  ///< maximum number of iterations
  size_t _minIterations;  ///< minimum number of iterations
  bool _systemInited;     ///< initialization flag

  float _deltaTAbort;  ///< optimization abort threshold for deltaT
  float _deltaRAbort;  ///< optimization abort threshold for deltaR

  pcl::PointCloud<pcl::PointXYZI>::Ptr _lastCornerCloud;   ///< last corner points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _lastSurfaceCloud;  ///< last surface points cloud

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudOri;  ///< point selection
  pcl::PointCloud<pcl::PointXYZI>::Ptr _coeffSel;       ///< point selection coefficients

  nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastCornerKDTree;   ///< last corner cloud KD-tree
  nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastSurfaceKDTree;  ///< last surface cloud KD-tree

  pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsSharp;      ///< sharp corner points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsLessSharp;  ///< less sharp corner points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsFlat;         ///< flat surface points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsLessFlat;     ///< less flat surface points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloud;             ///< full resolution cloud

  std::vector<int> _pointSearchCornerInd1;  ///< first corner point search index buffer
  std::vector<int> _pointSearchCornerInd2;  ///< second corner point search index buffer

  std::vector<int> _pointSearchSurfInd1;  ///< first surface point search index buffer
  std::vector<int> _pointSearchSurfInd2;  ///< second surface point search index buffer
  std::vector<int> _pointSearchSurfInd3;  ///< third surface point search index buffer

  Twist _transform;     ///< optimized pose transformation //smk: also used as motion prior, also adjusted by IMU or VIO(if needed)
  Twist _transformSum;  ///< accumulated optimized pose transformation

  Angle _imuRollStart, _imuPitchStart, _imuYawStart;
  Angle _imuRollEnd, _imuPitchEnd, _imuYawEnd;

  Vector3 _imuShiftFromStart;
  Vector3 _imuVeloFromStart;

  //CUSTOMIZATION
  std::ofstream _odometryLogFile;           // logging file for debugging
  float _odomDegenEigVal;                   // optimization degeneracy checking eigen value threshold
  double _odomRelativeTranslationMax;       // external odometry translation prior norm threshold(e.g. allowed max velocity is 1m/s so at 10Hz of Laserodometry max distance can be 0.1m (0.125 with some tolerance))
  double _odomRelativeRotationMax;          // external odometry rotation prior norm threshold, in radians per (1/scan-rate)
  ros::Time _currentPCLTime, _lastPCLTime;  // timestamps of last and current pointclouds
  bool _wasDegenerate = false;              // flag to check if laser odometry was degnerate in the past
  int _countDegenerate = 0;                 // laser odometry degeneracy counter
  int _verboseLevel = 0;

  //External Motion Prior Parameters
  tf::TransformListener _tfListener;         // Transform Listener for calculating external estimate
  bool _isExternalPriorHealthy = false;      // Flag to check if external prior is of good quality
  bool _extPriorAvailable = false;           // Flag to check if 'Primary' external prior is available
  std::string _extOdomFrame = "";            // External Prior odometry frame name
  std::string _extFixedFrame = "";           // External Prior fixed frame name
  std::string _extSensorFrame = "";          // External Prior sensor frame name
  double _extOdomTimeOffset = 0.0;           // Timeoffset between LiDAR pointcloud and external source
  bool _fallbackExtPriorAvailable = false;   // Flag to check if 'Fallback' external prior is available
  std::string _fallbackExtOdomFrame = "";    // Fallback External Prior odometry frame name
  std::string _fallbackExtFixedFrame = "";   // Fallback External Prior fixed frame name
  std::string _fallbackExtSensorFrame = "";  // Fallback External Prior sensor frame name
  double _fallbackExtOdomTimeOffset = 0.0;   // Timeoffset between LiDAR pointcloud and Fallbackexternal source
  std::string _lidarFrame = "";              // LiDAR frame name - used to lookup LiDAR-to-ExternalSensor Frame
  bool _isInputPCLRotated = false;           // Flag indicating if input PCL is rotated due to mounting position of LiDAR
  tf::Transform _Rot_inCloudLiDAR;           // Rotation of input PCL w.r.t ROS frmae
  bool _undistortInputCloud = true;          // If set to true External Prior or Motion Model will be used for LiDAR Ego Motion Compensation of input cloud

  //Graph Parameters
  std::string _imuFrame = "";                           //IMU frame name - used to lookup LiDAR-to-IMU Frame
  double _imuTimeOffset = 0.0;                          //Offset between IMU and LiDAR Measurements - Depending on LiDAR timestamp first(+0.05) or last(-0.05)
  Eigen::Matrix4d _T_LB = Eigen::Matrix4d::Identity();  //IMU to LiDAR expressed in IMU frame, Read as Transforms LiDAR into IMU
  Eigen::Matrix4d _T_BL = Eigen::Matrix4d::Identity();  //LiDAR to IMU expressed in LiDAR frame, Read as Transforms IMU into LiDAR
  bool _initGraphRollPicthFromIMU = true;               //Initialize graph roll/pitch from IMU  
  bool _gravityAttitudeInit = false;                    //Flag if attitude from gravity were initialized
  tf::Transform _gravityAttitude;                       //Attitude from initial gravity alignment
  sensor_msgs::Imu _imuBiasMsg;                         //IMU bias publishing ROS message
  bool _zeroMotionDetection = false;                    //Detect and Add Zero Motion Factors(Zero delta Pose and Velocity)
  bool _gravityRollPitchFactors = false;                //Add Gravity-aligned Roll-Pitch from IMU when in Zero motion (only works if Zero-Motion Factors are added)
  //CUSTOMIZATION
};

}  // end namespace loam
#endif
