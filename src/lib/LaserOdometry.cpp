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

#include "loam/LaserOdometry.h"

#include <pcl/filters/filter.h>

#include "loam/common.h"


namespace loam {

using std::asin;
using std::atan2;
using std::cos;
using std::fabs;
using std::pow;
using std::sin;
using std::sqrt;

LaserOdometry::LaserOdometry(float scanPeriod, uint16_t ioRatio, size_t maxIterations)
    : BasicLaserOdometry(scanPeriod, maxIterations),
      _ioRatio(ioRatio) {
  // initialize tf messages
  _laserOdometryTrans.frame_id_ = "camera_init";
  _laserOdometryTrans.child_frame_id_ = "laser_odom";
  _cameraFrameId = "camera";
  _laserOdometryTrans_CORRECTED.frame_id_ = "camera_init_CORRECTED";
  _laserOdometryTrans_CORRECTED.child_frame_id_ = "laser_odom_CORRECTED";

  // initialize gravity aligned tf message
  _cameraInitGravityAlignedTf.frame_id_ = "camera_init_CORRECTED";
  _cameraInitGravityAlignedTf.child_frame_id_ = "camera_init_GRAVITY_ALIGNED";
  _cameraInitGravityAlignedTf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
}

bool LaserOdometry::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  // fetch laser odometry params
  double dParam;
  float fParam;
  int iParam;
  bool bParam;
  std::string sParam;
  std::vector<double> dvParam;

  if (privateNode.getParam("scanPeriod", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      setScanPeriod(fParam);
      ROS_INFO("Set scanPeriod: %g", fParam);
    }
  }

  if (privateNode.getParam("ioRatio", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid ioRatio parameter: %d (expected > 0)", iParam);
      return false;
    } else {
      _ioRatio = iParam;
      ROS_INFO("Set ioRatio: %d", iParam);
    }
  }

  if (privateNode.getParam("odomMaxIterations", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid odomMaxIterations parameter: %d (expected > 0)", iParam);
      return false;
    } else {
      setMaxIterations(iParam);
      ROS_INFO("Set odomMaxIterations: %d", static_cast<int>(maxIterations()));
    }
  }

  if (privateNode.getParam("odomMinIterations", iParam)) {
    if (iParam < 0) {
      ROS_ERROR("Invalid odomMinIterations parameter: %d", iParam);
      return false;
    } else {
      setMinIterations(iParam);
      ROS_INFO("Set odomMinIterations: %d", static_cast<int>(minIterations()));
    }
  }

  if (privateNode.getParam("odomDeltaTAbort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid odomDeltaTAbort parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      setDeltaTAbort(fParam);
      ROS_INFO("Set odomDeltaTAbort: %g", fParam);
    }
  }

  if (privateNode.getParam("odomDeltaRAbort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid odomDeltaRAbort parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      setDeltaRAbort(fParam);
      ROS_INFO("Set odomDeltaRAbort: %g", fParam);
    }
  }

  //CUSTOMIZATION
  // Odometry Optimization Degenerate EigenValue
  if (privateNode.getParam("odomDegenEigVal", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid odomDegenEigVal parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      setDegenEigenThreshold(fParam);
      ROS_INFO("Set odomDegenEigVal: %g", fParam);
    }
  }

  //Odometry Translation Prior Max
  if (privateNode.getParam("odomRelativeTranslationMax", dParam)) {
    if (dParam <= 0) {
      ROS_ERROR("Invalid odomRelativeTranslationMax parameter: %f (expected > 0)", dParam);
      return false;
    } else {
      setExtOdomRelativeTranslationMax(dParam);
      ROS_INFO("Set odomRelativeTranslationMax: %g", dParam);
    }
  }

  //Odometry Rotation Prior Max
  if (privateNode.getParam("odomRelativeRotationMax", dParam)) {
    if (dParam <= 0) {
      ROS_ERROR("Invalid odomRelativeRotationMax parameter: %f (expected > 0)", dParam);
      return false;
    } else {
      setExtOdomRelativeRotationMax(dParam);
      ROS_INFO("Set odomRelativeRotationMax: %g", dParam);
    }
  }

  //External Prior
  if (privateNode.getParam("extPriorAvailable", bParam)) {
    setExternalPriorStatus(bParam);
    ROS_INFO_STREAM("LaserOdometry - External Prior Available: " << (bParam ? "True" : "False"));
    if (bParam) {
      //Set external ODOMETRY frame
      if (privateNode.getParam("extOdomFrame", sParam)) {
        ROS_INFO_STREAM("LaserOdometry - external ODOMETRY frame: " << sParam);
        setExternalOdometryFrame(sParam);
      } else {
        ROS_WARN("LaserOdometry - external ODOMETRY frame not set - DISABLING USE OF EXTERNAL PRIOR");
        setExternalPriorStatus(false);
      }

      //Set external FIXED frame
      if (privateNode.getParam("extFixedFrame", sParam)) {
        ROS_INFO_STREAM("LaserOdometry - external FIXED frame: " << sParam);
        setExternalFixedFrame(sParam);
      } else {
        ROS_WARN("LaserOdometry - external FIXED frame not set - DISABLING USE OF EXTERNAL PRIOR");
        setExternalPriorStatus(false);
      }

      //Set external SENSOR frame
      if (privateNode.getParam("extSensorFrame", sParam)) {
        ROS_INFO_STREAM("LaserOdometry - external SENSOR frame: " << sParam);
        setExternalSensorFrame(sParam);
      } else {
        ROS_WARN("LaserOdometry - external SENSOR frame not set - DISABLING USE OF EXTERNAL PRIOR");
        setExternalPriorStatus(false);
      }

      //Set external odometry time offset
      dParam = 0.0;
      if (privateNode.getParam("extOdomTimeOffset", dParam))
        setExternalOdometryTimeOffset(dParam);
      ROS_INFO_STREAM("LaserOdometry - external odometry time offset[s]: " << dParam);
    }
  }

  //Fallback External Prior
  if (privateNode.getParam("fallbackExtPriorAvailable", bParam)) {
    setFallbackExternalPriorStatus(bParam);
    ROS_INFO_STREAM("LaserOdometry - Fallback External Prior Available: " << (bParam ? "True" : "False"));
    if (bParam) {
      //Set external ODOMETRY frame
      if (privateNode.getParam("fallbackExtOdomFrame", sParam)) {
        ROS_INFO_STREAM("LaserOdometry - fallback external ODOMETRY frame: " << sParam);
        setFallbackExternalOdometryFrame(sParam);
      } else {
        ROS_WARN("LaserOdometry - fallback external ODOMETRY frame not set - DISABLING USE OF FALLBACK EXTERNAL PRIOR");
        setFallbackExternalPriorStatus(false);
      }

      //Set external FIXED frame
      if (privateNode.getParam("fallbackExtFixedFrame", sParam)) {
        ROS_INFO_STREAM("LaserOdometry - fallback external FIXED frame: " << sParam);
        setFallbackExternalFixedFrame(sParam);
      } else {
        ROS_WARN("LaserOdometry - fallback external FIXED frame not set - DISABLING USE OF FALLBACK EXTERNAL PRIOR");
        setFallbackExternalPriorStatus(false);
      }

      //Set external SENSOR frame
      if (privateNode.getParam("fallbackExtSensorFrame", sParam)) {
        ROS_INFO_STREAM("LaserOdometry - fallback external SENSOR frame: " << sParam);
        setFallbackExternalSensorFrame(sParam);
      } else {
        ROS_WARN("LaserOdometry - fallback external SENSOR frame not set - DISABLING USE OF FALLBACK EXTERNAL PRIOR");
        setFallbackExternalPriorStatus(false);
      }

      //Set fallback external odometry time offset
      dParam = 0.0;
      if (privateNode.getParam("fallbackExtOdomTimeOffset", dParam))
        setFallbackExternalOdomTimeOffset(dParam);
      ROS_INFO_STREAM("LaserOdometry - fallback external odometry time offset[s]: " << dParam);
    }
  }

  //Set LiDAR frame
  if (privateNode.getParam("lidarFrame", sParam)) {
    ROS_INFO_STREAM("LaserOdometry - LiDAR frame: " << sParam);
    setLidarFrame(sParam);
  } else {
    ROS_WARN("LaserOdometry - LiDAR frame not set - DISABLING USE OF EXTERNAL PRIORS");
    setExternalPriorStatus(false);
    setFallbackExternalPriorStatus(false);
  }

  //Set input PCL rotation w.r.t LiDAR frame
  if (privateNode.getParam("rotateInputCloud", bParam)) {
    setInputPCLRotationStatus(bParam);
    ROS_INFO_STREAM("LaserOdometry - Rotate Input Pointcloud before using: " << (bParam ? "True" : "False"));
    if (bParam) {
      std::vector<double> ypr{0.0, 0.0, 0.0};
      if (privateNode.getParam("inputCloudRotation", ypr)) {
        if (ypr.size() == 3) {
          Eigen::Quaterniond q = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
          q.normalize();
          tf::Transform T(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(0, 0, 0)); /*quaternion order: x,y,z,w*/
          setLiDARtoInputPCLRotation(T);
          ROS_INFO_STREAM("LaserOdometry - Rotation applied to input clound in order YPR(rad): [" << ypr[0] << " " << ypr[1] << " " << ypr[2] << "]");
        } else {
          ROS_ERROR("LaserOdometry - Expected order Yaw-Pitch-Roll in radians");
          setInputPCLRotationStatus(false);
        }
      }
    }
  }

  //Set input cloud undistortion status - if distorted input then approx ego motion compensation will be performed using external prior(if available) or motion model
  if (privateNode.getParam("undistortInputCloud", bParam)) {
    setUndistortInputCloud(bParam);
    if(bParam)
      ROS_WARN_STREAM("LaserOdometry - Input Pointcloud is Distorted - External Prior/Motion Model will be use for Ego Motion Compensation");
    else
      ROS_WARN_STREAM("LaserOdometry - Input Pointcloud is Undistorted - NO Ego Motion Compensation will be performed");
  }

  /** Graph Parameters **/
  //IMU Preintegrtor
  if (privateNode.getParam("imuFrame", sParam)) {
    ROS_INFO_STREAM("LaserOdometry - IMU frame for preintegrator: " << sParam);
    setImuFrame(sParam);
  } else
    ROS_WARN("LaserOdometry - IMU frame not set for preintegrator");
  if (privateNode.getParam("imuTimeOffset", dParam)) {
    ROS_INFO_STREAM("LaserOdometry - IMU time offset[s]: " << dParam);
    setImuTimeOffset(dParam);
  }
  if (privateNode.getParam("imuRate", dParam)) {
    ROS_INFO_STREAM("LaserOdometry - IMU rate for preintegrator: " << dParam);
    _imuBuffer.setImuRate(dParam);
  }
  if (privateNode.getParam("imuTimeForInit", dParam)) {
    ROS_INFO_STREAM("LaserOdometry - IMU time used for preintegrator initilization: " << dParam);
    _imuBuffer.setImuTimeForInit(dParam);
  }
  if (privateNode.getParam("accNoiseDensity", dParam))
    _graphMgr.setAccNoiseDensity(dParam);
  if (privateNode.getParam("accBiasRandomWalk", dParam))
    _graphMgr.setAccBiasRandomWalk(dParam);
  if (privateNode.getParam("gyrNoiseDensity", dParam))
    _graphMgr.setGyroNoiseDensity(dParam);
  if (privateNode.getParam("gyrBiasRandomWalk", dParam))
    _graphMgr.setGyrBiasRandomWalk(dParam);
  if (privateNode.getParam("imuIntegrationCovariance", dParam))
    _graphMgr.setImuIntegrationCovariance(dParam);
  if (privateNode.getParam("imuBiasAccOmegaInt", dParam))
    _graphMgr.setImuBiasAccOmegaInt(dParam);
  if (privateNode.getParam("accBiasPrior", dvParam))
    _graphMgr.setAccBiasPrior(dvParam);
  if (privateNode.getParam("gyrBiasPrior", dvParam))
    _graphMgr.setGyrBiasPrior(dvParam);
  //Factor Graph
  if (privateNode.getParam("smootherLag", dParam))
    _graphMgr.setSmootherLag(dParam);
  if (privateNode.getParam("additonalIterations", iParam))
    _graphMgr.setIterations(iParam);
  if (privateNode.getParam("positionReLinTh", dParam))
    _graphMgr.setPositionReLinTh(dParam);
  if (privateNode.getParam("rotationReLinTh", dParam))
    _graphMgr.setRotationReLinTh(dParam);
  if (privateNode.getParam("velocityReLinTh", dParam))
    _graphMgr.setVelocityReLinTh(dParam);
  if (privateNode.getParam("accBiasReLinTh", dParam))
    _graphMgr.setAccBiasReLinTh(dParam);
  if (privateNode.getParam("gyrBiasReLinTh", dParam))
    _graphMgr.setGyrBiasReLinTh(dParam);
  if (privateNode.getParam("relinearizeSkip", iParam))
    _graphMgr._params.setRelinearizeSkip(iParam);
  if (privateNode.getParam("enableRelinearization", bParam))
    _graphMgr._params.setEnableRelinearization(bParam);
  if (privateNode.getParam("evaluateNonlinearError", bParam))
    _graphMgr._params.setEvaluateNonlinearError(bParam);
  if (privateNode.getParam("cacheLinearizedFactors", bParam))
    _graphMgr._params.setCacheLinearizedFactors(bParam);
  if (privateNode.getParam("findUnusedFactorSlots", bParam))
    _graphMgr._params.findUnusedFactorSlots = bParam;
  if (privateNode.getParam("enablePartialRelinearizationCheck", bParam))
    _graphMgr._params.setEnablePartialRelinearizationCheck(bParam);
  if (privateNode.getParam("enableDetailedResults", bParam))
    _graphMgr._params.setEnableDetailedResults(bParam);
  //Pose between factor noise
  if (privateNode.getParam("poseBetweenNoise", dvParam))
    _graphMgr.setPoseNoise(dvParam);
  //Zero Motion Factor & Gravity Roll/Pitch Factor
  if (privateNode.getParam("zeroMotionDetection", bParam)) {
    setZeroMotionDetection(bParam);
    if (bParam) {
      ROS_INFO_STREAM("LaserOdometry - Zero Motion Detection ENABLED - Zero Delta Pose & Velocity Factors added");
      //Zero motion detection threshold
      if (privateNode.getParam("zeroMotionThreshold", dParam)) {
        _graphMgr.setZeroMotionTh(dParam);
        ROS_INFO_STREAM("LaserOdometry - Zero Motion Detection Threshold(m): " << dParam);
      }
      //Minimum consective zero motion detection
      if (privateNode.getParam("minZeroMoitionDetections", iParam)) {
        _graphMgr.setMinZeroMotionDetections(iParam);
        ROS_INFO_STREAM("LaserOdometry - Min consective Zero Motion Detections: " << iParam);
      }
      //Check if Gravity-algined Roll-Pitch factor should be added
      if (privateNode.getParam("gravityRollPitchFactors", bParam)) {
        setUseGravityRollPitchFactors(bParam);
        if (bParam)
          ROS_INFO_STREAM("LaserOdometry - Gravity-aligned Roll-Pitch Factors added when in Zero Motion");
      }
    } else
      ROS_INFO_STREAM("LaserOdometry - Zero Motion Detection DISABLED");
  }
  // Initialize graph roll/pitch from IMU  
  if (privateNode.getParam("initGraphRollPicthFromIMU", bParam)) {
    setUseIMURollPitchForGraphInit(bParam);
    ROS_INFO_STREAM("LaserOdometry - Graph Roll/Pitch initialized from IMU: " << (bParam ? "True" : "False"));
    if (!bParam) {
      setUseGravityRollPitchFactors(bParam);
      ROS_INFO_STREAM("LaserOdometry - Zero Motion Detection DISABLED as Graph Roll/Pitch NOT initialized from IMU");
    }
  }
  /** Graph Parameters **/

  //Verbose Ouput
  iParam = 0;
  if (privateNode.getParam("loamVerbosity", iParam)) {
    ROS_INFO("Set loamVerbosity: %d", iParam);
    setVerboseLevel(iParam);
  } else
    setVerboseLevel(0);
  //CUSTOMIZATION

  // advertise laser odometry topics
  _pubLaserCloudCornerLast = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
  _pubLaserCloudSurfLast = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
  _pubLaserCloudFullRes = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 2);
  _pubLaserOdometry = node.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);

  // subscribe input Cloud topics using ROS exact sync policy in a single callback
  _subSharp.subscribe(node, "/laser_cloud_sharp", 2);
  _subLessSharp.subscribe(node, "/laser_cloud_less_sharp", 2);
  _subFlat.subscribe(node, "/laser_cloud_flat", 2);
  _subLessFlat.subscribe(node, "/laser_cloud_less_flat", 2);
  _subFull.subscribe(node, "/velodyne_cloud_2", 2);
  _pclExactSyncPtr.reset(new message_filters::Synchronizer<_pclExactSyncPolicy>(_pclExactSyncPolicy(10), _subSharp, _subLessSharp, _subFlat, _subLessFlat, _subFull));
  _pclExactSyncPtr->registerCallback(boost::bind(&LaserOdometry::syncCloudsHandler, this, _1, _2, _3, _4, _5));

  // subscribe to IMU topic
  _subImuTrans = node.subscribe<sensor_msgs::PointCloud2>("/imu_trans", 5, &LaserOdometry::imuTransHandler, this);
  _subImu = node.subscribe<sensor_msgs::Imu>("/imu", 400, &LaserOdometry::imuCallback, this, ros::TransportHints().tcpNoDelay());

  // ARL Coordinate Frame Facilitators
  _pubLaserOdometry_CORRECTED = node.advertise<nav_msgs::Odometry>("/laser_odom_to_init_CORRECTED", 5);
  // END ARL Coordinate Frame Facilitators

  //CUSTOMIZATION
  _pubOptimizationStatus = node.advertise<OptStatus>("/laser_odom_optimization_status", 5);
  _pubLaserOdometryStatus = node.advertise<std_msgs::Header>("/laser_odom_status", 5);
  _pubLaserImuBias = node.advertise<sensor_msgs::Imu>("/laser_odom_imu_bias", 20);
  //CUSTOMIZATION
  return true;
}  // namespace loam

void LaserOdometry::laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg) {
  _timeCornerPointsSharp = cornerPointsSharpMsg->header.stamp;

  cornerPointsSharp()->clear();
  pcl::fromROSMsg(*cornerPointsSharpMsg, *cornerPointsSharp());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cornerPointsSharp(), *cornerPointsSharp(), indices);
}

void LaserOdometry::laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg) {
  _timeCornerPointsLessSharp = cornerPointsLessSharpMsg->header.stamp;

  cornerPointsLessSharp()->clear();
  pcl::fromROSMsg(*cornerPointsLessSharpMsg, *cornerPointsLessSharp());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cornerPointsLessSharp(), *cornerPointsLessSharp(), indices);
}

void LaserOdometry::laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg) {
  _timeSurfPointsFlat = surfPointsFlatMsg->header.stamp;

  surfPointsFlat()->clear();
  pcl::fromROSMsg(*surfPointsFlatMsg, *surfPointsFlat());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*surfPointsFlat(), *surfPointsFlat(), indices);
}

void LaserOdometry::laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg) {
  _timeSurfPointsLessFlat = surfPointsLessFlatMsg->header.stamp;

  surfPointsLessFlat()->clear();
  pcl::fromROSMsg(*surfPointsLessFlatMsg, *surfPointsLessFlat());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*surfPointsLessFlat(), *surfPointsLessFlat(), indices);
}

void LaserOdometry::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg) {
  _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;

  laserCloud()->clear();
  pcl::fromROSMsg(*laserCloudFullResMsg, *laserCloud());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloud(), *laserCloud(), indices);
}

void LaserOdometry::imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg) {
  _timeImuTrans = imuTransMsg->header.stamp;

  pcl::PointCloud<pcl::PointXYZ> imuTrans;
  pcl::fromROSMsg(*imuTransMsg, imuTrans);
  updateIMU(imuTrans);
}

void LaserOdometry::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr) {
  //Add to buffer
  _imuBuffer.addToIMUBuffer(imu_ptr->header.stamp.toSec(),
                            imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z,
                            imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z);
}

void LaserOdometry::syncCloudsHandler(const sensor_msgs::PointCloud2ConstPtr& pclSharp,
                                      const sensor_msgs::PointCloud2ConstPtr& pclLessSharp,
                                      const sensor_msgs::PointCloud2ConstPtr& pclFlat,
                                      const sensor_msgs::PointCloud2ConstPtr& pclLessFlat,
                                      const sensor_msgs::PointCloud2ConstPtr& pclFull) {
  //Process incoming pointclouds
  laserCloudSharpHandler(pclSharp);
  laserCloudLessSharpHandler(pclLessSharp);
  laserCloudFlatHandler(pclFlat);
  laserCloudLessFlatHandler(pclLessFlat);
  laserCloudFullResHandler(pclFull);

  // try processing new data
  process();
}

void LaserOdometry::process() {
  //Update Timestamp according to timestamp of original PCL
  updatePCLTime(_timeSurfPointsLessFlat);

  BasicLaserOdometry::process();
  publishResult();
}

void LaserOdometry::publishResult() {
  // publish odometry transformations
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum().rot_z.rad(),
                                                                              -transformSum().rot_x.rad(),
                                                                              -transformSum().rot_y.rad());

  // initialize odometry and odometry tf messages
  nav_msgs::OdometryPtr laserOdometryMsgPtr(new nav_msgs::Odometry);
  laserOdometryMsgPtr->header.frame_id = _laserOdometryTrans.frame_id_;
  laserOdometryMsgPtr->child_frame_id = _laserOdometryTrans.child_frame_id_;
  laserOdometryMsgPtr->header.stamp = _timeSurfPointsLessFlat;
  laserOdometryMsgPtr->pose.pose.orientation.x = -geoQuat.y;
  laserOdometryMsgPtr->pose.pose.orientation.y = -geoQuat.z;
  laserOdometryMsgPtr->pose.pose.orientation.z = geoQuat.x;
  laserOdometryMsgPtr->pose.pose.orientation.w = geoQuat.w;
  laserOdometryMsgPtr->pose.pose.position.x = transformSum().pos.x();
  laserOdometryMsgPtr->pose.pose.position.y = transformSum().pos.y();
  laserOdometryMsgPtr->pose.pose.position.z = transformSum().pos.z();
  _pubLaserOdometry.publish(laserOdometryMsgPtr);

  _laserOdometryTrans.stamp_ = _timeSurfPointsLessFlat;
  _laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  _laserOdometryTrans.setOrigin(tf::Vector3(transformSum().pos.x(), transformSum().pos.y(), transformSum().pos.z()));
  _tfBroadcaster.sendTransform(_laserOdometryTrans);

  // ARL Coordinate Frame Facilitators
  nav_msgs::OdometryPtr laserOdometry_CORRECTEDPtr(new nav_msgs::Odometry);
  laserOdometry_CORRECTEDPtr->header.frame_id = _laserOdometryTrans_CORRECTED.frame_id_;
  laserOdometry_CORRECTEDPtr->child_frame_id = _laserOdometryTrans_CORRECTED.child_frame_id_;
  laserOdometry_CORRECTEDPtr->header.stamp = _timeSurfPointsLessFlat;
  laserOdometry_CORRECTEDPtr->pose.pose.orientation.x = geoQuat.x;
  laserOdometry_CORRECTEDPtr->pose.pose.orientation.y = -geoQuat.y;
  laserOdometry_CORRECTEDPtr->pose.pose.orientation.z = -geoQuat.z;
  laserOdometry_CORRECTEDPtr->pose.pose.orientation.w = geoQuat.w;
  laserOdometry_CORRECTEDPtr->pose.pose.position.x = transformSum().pos.z();  //smk: LOAM doesn't follow camera convention but Z-Forward, X-Left and Y-Up
  laserOdometry_CORRECTEDPtr->pose.pose.position.y = transformSum().pos.x();
  laserOdometry_CORRECTEDPtr->pose.pose.position.z = transformSum().pos.y();
  _pubLaserOdometry_CORRECTED.publish(laserOdometry_CORRECTEDPtr);

  _laserOdometryTrans_CORRECTED.stamp_ = _timeSurfPointsLessFlat;
  _laserOdometryTrans_CORRECTED.setRotation(tf::Quaternion(laserOdometry_CORRECTEDPtr->pose.pose.orientation.x,
                                                           laserOdometry_CORRECTEDPtr->pose.pose.orientation.y,
                                                           laserOdometry_CORRECTEDPtr->pose.pose.orientation.z,
                                                           laserOdometry_CORRECTEDPtr->pose.pose.orientation.w));
  _laserOdometryTrans_CORRECTED.setOrigin(tf::Vector3(laserOdometry_CORRECTEDPtr->pose.pose.position.x,
                                                      laserOdometry_CORRECTEDPtr->pose.pose.position.y,
                                                      laserOdometry_CORRECTEDPtr->pose.pose.position.z));
  _tfBroadcaster.sendTransform(_laserOdometryTrans_CORRECTED);
  // END ARL Coordinate Frame Facilitators

  //IMU Bias
  if (_pubLaserImuBias.getNumSubscribers() > 0) {
    sensor_msgs::Imu imuBiasMsg;
    imuBiasMsg.header.frame_id = "/laser_odometry_imu";
    imuBiasMsg.header.stamp = _timeSurfPointsLessFlat;
    imuBiasMsg.linear_acceleration.x = graphIMUBias().linear_acceleration.x;
    imuBiasMsg.linear_acceleration.y = graphIMUBias().linear_acceleration.y;
    imuBiasMsg.linear_acceleration.z = graphIMUBias().linear_acceleration.z;
    imuBiasMsg.angular_velocity.x = graphIMUBias().angular_velocity.x;
    imuBiasMsg.angular_velocity.y = graphIMUBias().angular_velocity.y;
    imuBiasMsg.angular_velocity.z = graphIMUBias().angular_velocity.z;
    _pubLaserImuBias.publish(imuBiasMsg);
  }
  //CUSTOMIZATION
  //Publish LaserOdometry Optimization Status
  if (_pubOptimizationStatus.getNumSubscribers() > 0) {
    _optStatusMsg.header.stamp = _timeSurfPointsLessFlat;
    _pubOptimizationStatus.publish(_optStatusMsg);
  }
  //Publish LaserOdometry Status
  std_msgs::HeaderPtr laserOdomStatusMsgPtr(new std_msgs::Header);
  laserOdomStatusMsgPtr->stamp = _timeSurfPointsLessFlat;
  laserOdomStatusMsgPtr->seq = (getLaserOdometryStatus() ? 1 : 0);
  _pubLaserOdometryStatus.publish(laserOdomStatusMsgPtr);
  //Publish Gravity Aligned Init frame
  if (gravityAttitudeInitialized()) {
    _cameraInitGravityAlignedTf.stamp_ = _timeSurfPointsLessFlat;
    _cameraInitGravityAlignedTf.setBasis(getLiDARGravityInitAttitude().getBasis());
    _tfBroadcaster.sendTransform(_cameraInitGravityAlignedTf);
  }
  //CUSTOMIZATION

  // publish cloud results according to the input output ratio
  if (_ioRatio < 2 || frameCount() % _ioRatio == 1) {
    publishCloudMsg(_pubLaserCloudCornerLast, *lastCornerCloud(), _timeSurfPointsLessFlat, _cameraFrameId);  //smk: becuase of _ioRatio=2 this is 5Hz
    publishCloudMsg(_pubLaserCloudSurfLast, *lastSurfaceCloud(), _timeSurfPointsLessFlat, _cameraFrameId);
    transformToEnd(laserCloud());  // transform full resolution cloud to sweep end before sending it ///smk: velodyne_cloud_2 is transformed and sent out as /velodyne_cloud_3
    publishCloudMsg(_pubLaserCloudFullRes, *laserCloud(), _timeSurfPointsLessFlat, _cameraFrameId);
  }
}

}  // end namespace loam
