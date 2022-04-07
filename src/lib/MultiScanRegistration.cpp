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

#include "loam/MultiScanRegistration.h"

#include <pcl_conversions/pcl_conversions.h>



namespace loam {
MultiScanMapper::MultiScanMapper(const float& lowerBound,
                                 const float& upperBound,
                                 const uint16_t& nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound))  //smk:for VLP16 0.5
{
}

void MultiScanMapper::set(const float& lowerBound,
                          const float& upperBound,
                          const uint16_t& nScanRings) {
  _lowerBound = lowerBound;
  _upperBound = upperBound;
  _nScanRings = nScanRings;
  _factor = (nScanRings - 1) / (upperBound - lowerBound);
}

int MultiScanMapper::getRingForAngle(const float& angle) {
  return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}

MultiScanRegistration::MultiScanRegistration(const MultiScanMapper& scanMapper)
    : _scanMapper(scanMapper){};

bool MultiScanRegistration::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  RegistrationParams config;
  if (!setupROS(node, privateNode, config))
    return false;

  configure(config);
  return true;
}

bool MultiScanRegistration::setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out) {
  if (!ScanRegistration::setupROS(node, privateNode, config_out))
    return false;

  // fetch scan mapping params
  std::string lidarName;

  if (privateNode.getParam("lidar", lidarName)) {
    if (lidarName == "VLP-16" || "VLP-16-ANYbotics") {
      _scanMapper = MultiScanMapper::Velodyne_VLP_16();
    } else if (lidarName == "HDL-32") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_32();
    } else if (lidarName == "HDL-64E") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
    } else if (lidarName == "O1-64") {
      _scanMapper = MultiScanMapper::Ouster_O1_64();
    } else if (lidarName == "O1-16") {
      _scanMapper = MultiScanMapper::Ouster_O1_16();
    } else if (lidarName == "Bperl-32") {
      _scanMapper = MultiScanMapper::RS_Bperl_32();
    } else if (lidarName == "O0-128") {
      _scanMapper = MultiScanMapper::Ouster_O0_128();
    } else if (lidarName == "O0-128-Div2") {
      _scanMapper = MultiScanMapper::Ouster_O0_128_Div2();
    } else {
      ROS_ERROR("Invalid lidar parameter: model %s is not supported)", lidarName.c_str());
      return false;
    }
    ROS_INFO("Set  %s  scan mapper.", lidarName.c_str());

    if (!privateNode.hasParam("scanPeriod")) {
      config_out.scanPeriod = 0.1;
      ROS_INFO("Set scanPeriod: %f", config_out.scanPeriod);
    }
    //CUSTOMIZATION
    _scanMapper.setLiDARName(lidarName);
    //CUSTOMIZATION
  } else {
    float vAngleMin, vAngleMax;
    int nScanRings;

    if (privateNode.getParam("minVerticalAngle", vAngleMin) &&
        privateNode.getParam("maxVerticalAngle", vAngleMax) &&
        privateNode.getParam("nScanRings", nScanRings)) {
      if (vAngleMin >= vAngleMax) {
        ROS_ERROR("Invalid vertical range (min >= max)");
        return false;
      } else if (nScanRings < 2) {
        ROS_ERROR("Invalid number of scan rings (n < 2)");
        return false;
      }

      _scanMapper.set(vAngleMin, vAngleMax, nScanRings);
      ROS_INFO("Set linear scan mapper from %g to %g degrees with %d scan rings.", vAngleMin, vAngleMax, nScanRings);
    }
  }

  //CUTOMIZATION
  // Rotate input cloud before estimating odometry/map so produced resuts are ROS frame aligned irrespective of LiDAR mounting orientation
  if (privateNode.getParam("rotateInputCloud", _rotateInputCloud)) {
    ROS_INFO_STREAM("MultiScanRegistration - Rotate Input Pointcloud before using: " << (_rotateInputCloud ? "True" : "False"));
    if (_rotateInputCloud) {
      std::vector<double> ypr{0.0, 0.0, 0.0};
      if (privateNode.getParam("inputCloudRotation", ypr)) {
        if (ypr.size() == 3) {
          Eigen::Quaterniond q = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
          q.normalize();
          _affine.rotate(q);
          ROS_INFO_STREAM("MultiScanRegistration - Rotation applied to input clound in order YPR(rad): [" << ypr[0] << " " << ypr[1] << " " << ypr[2] << "]");
        } else {
          ROS_ERROR("MultiScanRegistration - Expected order Yaw-Pitch-Roll in radians");
          _rotateInputCloud = false;
        }
      }
    }
  }

  //Flag to use input pointcloud intensity or ring fields. Converts to custom PointXYZIR instead of PointXYZ
  if (privateNode.getParam("useCloudIntensityandRingFields", _useCloudIntensityandRingFields))
    ROS_INFO_STREAM("MultiScanRegistration - Using Input Clouds Intensity and Ring Fields: " << (_useCloudIntensityandRingFields ? "True" : "False"));


  //Get Min/Max useful range for coin
  privateNode.getParam("minRange", _minRange);
  privateNode.getParam("maxRange", _maxRange);
  ROS_INFO_STREAM("MultiScanRegistration - Minimum Range: " << _minRange);
  ROS_INFO_STREAM("MultiScanRegistration - Maximum Range: " << _maxRange);
  //CUTOMIZATION

  // subscribe to input cloud topic
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>("/multi_scan_points", 2, &MultiScanRegistration::handleCloudMessage, this);

  return true;
}

void MultiScanRegistration::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
  if (_systemDelay > 0) {
    --_systemDelay;
    return;
  }

  //Preserve ROS timestamp
  updatedROSPCLTimestamp(laserCloudMsg->header.stamp);

  //Check if OUster
  if (_scanMapper.getLiDARName() == "O0-128-Div2" ||
      _scanMapper.getLiDARName() == "O0-128" ||
      _scanMapper.getLiDARName() == "O1-64") {
    // Convert ROS Pointcloud2 into PCL Pointcloud<PointOuster>
    pcl::PointCloud<PointOuster> pclPointOuster;
    pcl::fromROSMsg(*laserCloudMsg, pclPointOuster);
    // process
    process(pclPointOuster, fromROSTime(laserCloudMsg->header.stamp));
  } else if (_scanMapper.getLiDARName() == "VLP-16-ANYbotics") {
    // Convert ROS Pointcloud2 into PCL Pointcloud<pclPointVelodyneANYbotics>
    pcl::PointCloud<PointVelodyneANYbotics> pclPointVelodyneANYbotics;
    pcl::fromROSMsg(*laserCloudMsg, pclPointVelodyneANYbotics);
    // process
    process(pclPointVelodyneANYbotics, fromROSTime(laserCloudMsg->header.stamp));
  } else {  //TODO - Currently treating all other inputs in Velodyne format
    // Convert ROS Pointcloud2 into PCL Pointcloud<PointVelodyne>
    pcl::PointCloud<PointVelodyne> pclPointVelodyne;
    pcl::fromROSMsg(*laserCloudMsg, pclPointVelodyne);
    // process
    process(pclPointVelodyne, fromROSTime(laserCloudMsg->header.stamp));
  }
}

template <typename pointType>
void MultiScanRegistration::process(const pcl::PointCloud<pointType>& laserCloudInput, const Time& scanTime) {
  //smk:
  //1. get cloud message and calculate start and end orientation of the scan
  //2. eliminate infinite and zero points
  //3. calculate vertical angle of point to determine scan ring (scanID), Y-axis seems to facing upward
  //4. calculates relative time from start of scan to look-up corresponding IMU message to refine point position in cloud as pcl is not collected all at once, especially true for rotating hokoyu lidars
  //5. perform point "ROTATION" only using IMU in YXZ order
  //6. LIDAR frame seems to be Z-Forward, X-Left and Y-Up (equivalent to ZYX of ros frame)

  //CUSTOMIZATION
  pcl::PointCloud<pointType> laserCloudIn;
  // Rotate input pointcloud
  if (_rotateInputCloud)
    pcl::transformPointCloud(laserCloudInput, laserCloudIn, _affine);
  else
    pcl::copyPointCloud(laserCloudInput, laserCloudIn);
  //CUSTOMIZATION

  size_t cloudSize = laserCloudIn.size();
  if (cloudSize == 0) {
    ROS_WARN("LOAM received an empty PCL.");
    return;
  }

  // determine scan start and end orientations
  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y, laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  pcl::PointXYZI point;
  _laserCloudScans.resize(_scanMapper.getNumberOfScanRings());
  // clear all scanline points
  std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(), [](auto&& v) { v.clear(); });

  // extract valid points from input cloud
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;

    // skip NaN and INF valued points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      continue;

    //Range
    float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    // Skip points outside of useful range
    if (range < _minRange || range > _maxRange)
      continue;

    //Use ring number or calculate elevation angle to determine scanID
    int scanID;
    if (_scanMapper.getLiDARName() == "O0-128-Div2" || _scanMapper.getLiDARName() == "O0-128")
      scanID = static_cast<int>(laserCloudIn[i].ring);
    else {
      float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));  //TODO: Check Formula
      scanID = _scanMapper.getRingForAngle(angle);
    }

    //Skip rings option incase of OS0-128 LiDAR
    if (_scanMapper.getLiDARName() == "O0-128-Div2" && scanID % 2 != 0)
      continue;

    //Skip point if outside elevation angle range of LiDAR
    if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0)
      continue;

    // calculate horizontal point angle
    float ori = -std::atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    // calculate relative scan time based on point orientation
    float relTime = config().scanPeriod * (ori - startOri) / (endOri - startOri);  //smk: given the orientation of the current point and knowning the starting,ending orientations and total scan time we can find the relative time at which this point was scanned when the lidar was doing its sweep or scan.
    point.intensity = scanID + relTime;
    //std::cout << "point.intensity: " << point.intensity << ", scanID: " << scanID << ", relTime:" << relTime << std::endl; //smk:never greater the 0-15 and 0.1

    projectPointToStartOfSweep(point, relTime);  //smk: this corrects point locations according to intermediate imu locations to make the pcl more consistent.

    _laserCloudScans[scanID].push_back(point);
  }

  processScanlines(scanTime, _laserCloudScans);  //smk: scanTime = (laserCloudMsg->header.stamp)
  publishResult();
}

template void MultiScanRegistration::process(const pcl::PointCloud<PointVelodyne>&, const Time&);
template void MultiScanRegistration::process(const pcl::PointCloud<PointOuster>&, const Time&);
template void MultiScanRegistration::process(const pcl::PointCloud<PointVelodyneANYbotics>&, const Time&);
}  // end namespace loam
