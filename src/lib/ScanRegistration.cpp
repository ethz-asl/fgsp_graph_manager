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

#include "loam/ScanRegistration.h"

#include <tf/transform_datatypes.h>


namespace loam {
bool ScanRegistration::parseParams(const ros::NodeHandle& nh, RegistrationParams& config_out) {
  bool success = true;
  int iParam = 0;
  float fParam = 0;
  bool bParam = false;
  std::vector<double> dvParam;

  if (nh.getParam("scanPeriod", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
      success = false;
    } else {
      config_out.scanPeriod = fParam;
      ROS_INFO("Set scanPeriod: %g", fParam);
    }
  }

  if (nh.getParam("imuHistorySize", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid imuHistorySize parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
      config_out.imuHistorySize = iParam;
      ROS_INFO("Set imuHistorySize: %d", iParam);
    }
  }

  if (nh.getParam("featureRegions", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid featureRegions parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
      config_out.nFeatureRegions = iParam;
      ROS_INFO("Set nFeatureRegions: %d", iParam);
    }
  }

  if (nh.getParam("curvatureRegion", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid curvatureRegion parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
      config_out.curvatureRegion = iParam;
      ROS_INFO("Set curvatureRegion: +/- %d", iParam);
    }
  }

  if (nh.getParam("maxCornerSharp", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid maxCornerSharp parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
      config_out.maxCornerSharp = iParam;
      config_out.maxCornerLessSharp = 10 * iParam;
      ROS_INFO("Set maxCornerSharp / less sharp: %d / %d", iParam, config_out.maxCornerLessSharp);
    }
  }

  if (nh.getParam("maxCornerLessSharp", iParam)) {
    if (iParam < config_out.maxCornerSharp) {
      ROS_ERROR("Invalid maxCornerLessSharp parameter: %d (expected >= %d)", iParam, config_out.maxCornerSharp);
      success = false;
    } else {
      config_out.maxCornerLessSharp = iParam;
      ROS_INFO("Set maxCornerLessSharp: %d", iParam);
    }
  }

  if (nh.getParam("maxSurfaceFlat", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid maxSurfaceFlat parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
      config_out.maxSurfaceFlat = iParam;
      ROS_INFO("Set maxSurfaceFlat: %d", iParam);
    }
  }

  if (nh.getParam("surfaceCurvatureThreshold", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("Invalid surfaceCurvatureThreshold parameter: %f (expected >= 0.001)", fParam);
      success = false;
    } else {
      config_out.surfaceCurvatureThreshold = fParam;
      ROS_INFO("Set surfaceCurvatureThreshold: %g", fParam);
    }
  }

  if (nh.getParam("lessFlatFilterSize", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("Invalid lessFlatFilterSize parameter: %f (expected >= 0.001)", fParam);
      success = false;
    } else {
      config_out.lessFlatFilterSize = fParam;
      ROS_INFO("Set lessFlatFilterSize: %g", fParam);
    }
  }

  // CUSTOMIZATION
  if (nh.getParam("checkInvalidFeatureRegions", bParam)) {
    config_out.checkInvalidFeatureRegions = bParam;
    if (bParam) {
      ROS_INFO("Check for Invalid Feature Regions - ENABLED");
      //Get Valid Azimuth region boundries
      if (nh.getParam("azimuthBoundaries", dvParam)) {
        if (dvParam.size() % 2 > 0) {
          ROS_ERROR("Invalid number of feature azimuthBoundaries set: %d, expected even number of inputs", int(dvParam.size()));
          success = false;
        } else {
          config_out.noFeatureAzimuthBoundaries = dvParam;
        }
      }
      //Get Valid Ploar region boundries
      if (nh.getParam("polarBoundaries", dvParam)) {
        if (dvParam.size() % 2 > 0) {
          ROS_ERROR("Invalid number of feature polarBoundaries set: %d, expected even number of inputs", int(dvParam.size()));
          success = false;
        } else {
          config_out.noFeaturePolarBoundaries = dvParam;
        }
      }
      //Invalid feaures visualization for debugging
      if (nh.getParam("publishDebugArrowsToRemovedFeatures", bParam)) {
        config_out.publishDebugArrowsToRemovedFeatures = bParam;
        if (bParam)
          ROS_INFO("Invalid Feature Visualization - ENABLED");
        else
          ROS_INFO("Invalid Feature Visualization - DISBALED");
      }
    } else
      ROS_WARN("Check for Invalid Feature Regions - DISBALED");
  }
  // CUSTOMIZATION

  return success;
}

bool ScanRegistration::setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out) {
  if (!parseParams(privateNode, config_out))  //smk: all parameters set here
    return false;

  // subscribe to IMU topic
  // _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &ScanRegistration::handleIMUMessage, this);

  // advertise scan registration topics
  _pubLaserCloud = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);
  _pubCornerPointsSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  _pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  _pubSurfPointsFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  _pubSurfPointsLessFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  _pubImuTrans = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);
  _pubInvalidFeaturesMarkers = node.advertise<visualization_msgs::MarkerArray>("/invalid_feature_markers", 1); //For Debugging only

  //Publishing frame name
  _cameraFrameId= "camera";
  return true;
}

void ScanRegistration::handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn) {
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  double roll, pitch, yaw;
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);  //smk: in IMU coordinates

  //smk: LOAM coordinates Z-Forward, X-Left and Y-Up, hence IMU acceleration axis need to be swaped, rotation is swaped in updateIMUData

  Vector3 acc;
  acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);  //smk: In LOAM coordinates
  acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);
  acc.z() = float(imuIn->linear_acceleration.x + sin(pitch) * 9.81);

  IMUState newState;
  newState.stamp = fromROSTime(imuIn->header.stamp);
  newState.roll = roll;
  newState.pitch = pitch;
  newState.yaw = yaw;
  newState.acceleration = acc;

  updateIMUData(acc, newState);
}

void ScanRegistration::publishResult() {
  auto sweepStartTime = _rosPCLTimeStamp;  //smk: publish at same ros time stamp as input pcl cloud

  // publish full resolution and feature point clouds
  publishCloudMsg(_pubLaserCloud, laserCloud(), sweepStartTime, _cameraFrameId);  //smk: contains all points but now ring-wise sorted
  publishCloudMsg(_pubCornerPointsSharp, cornerPointsSharp(), sweepStartTime, _cameraFrameId);
  publishCloudMsg(_pubCornerPointsLessSharp, cornerPointsLessSharp(), sweepStartTime, _cameraFrameId);
  publishCloudMsg(_pubSurfPointsFlat, surfacePointsFlat(), sweepStartTime, _cameraFrameId);
  publishCloudMsg(_pubSurfPointsLessFlat, surfacePointsLessFlat(), sweepStartTime, _cameraFrameId);

  // publish corresponding IMU transformation information
  publishCloudMsg(_pubImuTrans, imuTransform(), sweepStartTime, _cameraFrameId);

  // Publish Arrows for invalid features removed
  if (_pubInvalidFeaturesMarkers.getNumSubscribers() > 0)
    publishInvalidFeatureMarkers(invalidSharpFeatures());

}

void ScanRegistration::publishInvalidFeatureMarkers(const pcl::PointCloud<pcl::PointXYZI>& invalidFeatures) {
  visualization_msgs::MarkerArray arrows;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "camera";
  marker.ns = "invalidFeatureArrows";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  p1.x = 0.0;
  p1.y = 0.0;
  p1.z = 0.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.action = visualization_msgs::Marker::DELETEALL;
  arrows.markers.push_back(marker); 
  _pubInvalidFeaturesMarkers.publish(arrows); 
  arrows.markers.clear();

  marker.action = visualization_msgs::Marker::ADD;
   
  for (uint i = 0; i < invalidFeatures.size(); i+=10) {
    marker.points.clear();
    marker.id = i;
    marker.points.push_back(p1);
    p2.x = invalidFeatures[i].x;
    p2.y = invalidFeatures[i].y;
    p2.z = invalidFeatures[i].z;
    marker.points.push_back(p2);
    arrows.markers.push_back(marker);
  }

  _pubInvalidFeaturesMarkers.publish(arrows);
}

}  // end namespace loam
