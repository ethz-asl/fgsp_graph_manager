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

#include "loam/BasicLaserMapping.h"

#include <time.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include "loam/nanoflann_pcl.h"

namespace loam {

using std::asin;
using std::atan2;
using std::fabs;
using std::pow;
using std::sqrt;

BasicLaserMapping::BasicLaserMapping(const float& scanPeriod, const size_t& maxIterations)
    : _scanPeriod(scanPeriod),
      _stackFrameNum(1),
      _mapFrameNum(5),
      _frameCount(0),
      _mapFrameCount(0),
      _maxIterations(maxIterations),
      _minIterations(1),
      _deltaTAbort(0.05),
      _deltaRAbort(0.05),
      _mapDegenEigVal(100),
      _mapPriorNormThresh(0.3),
      _laserCloudCenWidth(10),
      _laserCloudCenHeight(5),
      _laserCloudCenDepth(10),
      _laserCloudWidth(21),
      _laserCloudHeight(11),
      _laserCloudDepth(21),
      _mapCubeSize(50.0),
      _neighborSubmapCubes(2),
      _laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudCornerStackDS(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurfStackDS(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurroundDS(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserMap(new pcl::PointCloud<pcl::PointXYZI>()),
      _submapSearchCloud(new pcl::PointCloud<pcl::PointXYZI>()) {
  // setup down size filters
  _downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
  _downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
  _downSizeFilterMapVis.setLeafSize(0.4, 0.4, 0.4);
  //Logging
  // _mappingLogFile.open("/home/shehryar/compslam_ws/src/compslam/KDTreeTimings.csv", std::ios::out);
  // _mappingLogFile << "CornerPts, SurfacePts, TotalPts, cubeKDtree-build(ms), Search\n";
}

void BasicLaserMapping::resetMapParamsAndContainers() {
  // initialize frame counter
  _frameCount = _stackFrameNum - 1;
  _mapFrameCount = _mapFrameNum - 1;

  // setup cloud vectors
  _laserCloudNum = _laserCloudWidth * _laserCloudHeight * _laserCloudDepth;
  _laserCloudCornerArray.clear();
  _laserCloudSurfArray.clear();
  _laserCloudCornerDSArray.clear();
  _laserCloudSurfDSArray.clear();
  _laserCloudCornerArray.resize(_laserCloudNum);
  _laserCloudSurfArray.resize(_laserCloudNum);
  _laserCloudCornerDSArray.resize(_laserCloudNum);
  _laserCloudSurfDSArray.resize(_laserCloudNum);

  for (size_t i = 0; i < _laserCloudNum; i++) {
    _laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    _laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    _laserCloudCornerDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    _laserCloudSurfDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  //CUSTOMIZATION
  //data structure for storing previosuly saved submaps
  _savedCornerSubmap.reset(new pcl::PointCloud<pcl::PointXYZI>());
  _savedSurfSubmap.reset(new pcl::PointCloud<pcl::PointXYZI>());
  //clear indices of occupied map cubes
  _occupiedCubeInd.clear();
  //CUSTOMIZATION

  std::cout << "======= LOAM MAP Parameters =======\n"
            << "Cube Size: " << _mapCubeSize << " meters"
            << "\nCube Neighbors used for creating Submap: +/-" << _neighborSubmapCubes
            << "\nTotal MAP Size (WxHxD): " << _laserCloudWidth << "x" << _laserCloudHeight << "x" << _laserCloudDepth << " cube units"
            << "\nRobot Start Position in Map (WxHxD): " << _laserCloudCenWidth << "x" << _laserCloudCenHeight << "x" << _laserCloudCenDepth << " cube units"
            << "\nMap Downsample Leaf Size, Corners: " << _downSizeFilterCorner.getLeafSize().transpose() << ", Surface: " << _downSizeFilterSurf.getLeafSize().transpose()
            << ", Map: " << _downSizeFilterMapVis.getLeafSize().transpose() << std::endl;
}

void BasicLaserMapping::transformAssociateToMap() {
  //ORIGINAL
  _transformIncre.pos = _transformBefMapped.pos - _transformSum.pos;
  rotateYXZ(_transformIncre.pos, -(_transformSum.rot_y), -(_transformSum.rot_x), -(_transformSum.rot_z));

  float sbcx = _transformSum.rot_x.sin();
  float cbcx = _transformSum.rot_x.cos();
  float sbcy = _transformSum.rot_y.sin();
  float cbcy = _transformSum.rot_y.cos();
  float sbcz = _transformSum.rot_z.sin();
  float cbcz = _transformSum.rot_z.cos();

  float sblx = _transformBefMapped.rot_x.sin();
  float cblx = _transformBefMapped.rot_x.cos();
  float sbly = _transformBefMapped.rot_y.sin();
  float cbly = _transformBefMapped.rot_y.cos();
  float sblz = _transformBefMapped.rot_z.sin();
  float cblz = _transformBefMapped.rot_z.cos();

  float salx = _transformAftMapped.rot_x.sin();
  float calx = _transformAftMapped.rot_x.cos();
  float saly = _transformAftMapped.rot_y.sin();
  float caly = _transformAftMapped.rot_y.cos();
  float salz = _transformAftMapped.rot_z.sin();
  float calz = _transformAftMapped.rot_z.cos();

  float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz) - cbcx * sbcy * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - cbcx * cbcy * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx);
  _transformTobeMapped.rot_x = -asin(srx);

  float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) - cblx * sblz * (caly * calz + salx * saly * salz) + calx * saly * sblx) - cbcx * cbcy * ((caly * calz + salx * saly * salz) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cbly * saly) + cbcx * sbcy * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * cblx * saly * sbly);
  float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) - cblx * cblz * (saly * salz + caly * calz * salx) + calx * caly * sblx) + cbcx * cbcy * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * caly * cblx * cbly) - cbcx * sbcy * ((saly * salz + caly * calz * salx) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (cbly * cblz + sblx * sbly * sblz) - calx * caly * cblx * sbly);
  _transformTobeMapped.rot_y = atan2(srycrx / _transformTobeMapped.rot_x.cos(),
                                     crycrx / _transformTobeMapped.rot_x.cos());

  float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) + cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) + cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  _transformTobeMapped.rot_z = atan2(srzcrx / _transformTobeMapped.rot_x.cos(),
                                     crzcrx / _transformTobeMapped.rot_x.cos());

  Vector3 v = _transformIncre.pos;
  rotateZXY(v, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);
  _transformTobeMapped.pos = _transformAftMapped.pos - v;

  //DEBUG
  if (_verboseLevel > 1)
    std::cout << "\033[32mMAPPING\033[0m Motion Model (x,y,z): [" << _transformTobeMapped.pos.z() << " " << _transformTobeMapped.pos.x() << " " << _transformTobeMapped.pos.y()
              << "], (r,p,y): [" << _transformTobeMapped.rot_z.deg() << " " << _transformTobeMapped.rot_x.deg() << " " << _transformTobeMapped.rot_y.deg() << "]\n";
  //DEBUG

  //CUSTOMIZATION
  //External Prior Integration
  bool priorSuceess = false;
  _isExternalPriorHealthy = false;
  Eigen::Vector3d translation_prior, rotation_prior;
  //Check if Primary Prior is available
  bool primaryPrior = false;
  if (_extPriorAvailable) {
    priorSuceess = calcExternalPrior(translation_prior, rotation_prior);
    //Check health
    if (priorSuceess && (translation_prior.norm() < _mapPriorNormThresh))
      _isExternalPriorHealthy = primaryPrior = true;
  }

  //Check if Fallback Prior is Available and Needed
  if (_fallbackExtPriorAvailable && !_isExternalPriorHealthy) {
    priorSuceess = calcExternalPrior(translation_prior, rotation_prior, true);
    //Check health
    if (priorSuceess && (translation_prior.norm() < _mapPriorNormThresh))
      _isExternalPriorHealthy = true;
  }

  //DEBUG
  if ((_extPriorAvailable || _fallbackExtPriorAvailable) && !_isExternalPriorHealthy)
    ROS_WARN_THROTTLE(1.0, "*** LaserMapping  - External Priors failed(x,y,z): [%f %f %f] - Motion Norm(>%f):%f ***", -translation_prior[2], -translation_prior[0], -translation_prior[1], _mapPriorNormThresh, translation_prior.norm());
  //DEBUG

  //Use Available HEALTHY Prior
  if (_isExternalPriorHealthy) {
    //Previos Pose - Convert from LOAM to ROS convention
    Eigen::Vector3d t_loam(_transformAftMapped.pos.z(), _transformAftMapped.pos.x(), _transformAftMapped.pos.y());
    Eigen::Quaterniond q_loam;
    q_loam = Eigen::AngleAxisd(_transformAftMapped.rot_y.rad(), Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(_transformAftMapped.rot_x.rad(), Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(_transformAftMapped.rot_z.rad(), Eigen::Vector3d::UnitX());

    //Increment - Convert from LOAM to ROS convention
    Eigen::Vector3d t_inc(translation_prior[2], translation_prior[0], translation_prior[1]);
    Eigen::Quaterniond q_inc;
    q_inc = Eigen::AngleAxisd(rotation_prior[1], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotation_prior[0], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotation_prior[2], Eigen::Vector3d::UnitX());

    //Update
    Eigen::Quaterniond q_up = q_loam * q_inc.inverse();  //LOCAL axis rotation
    Eigen::Vector3d t_up = t_loam - (q_loam * t_inc);

    //Get euler RPY angles
    tf::Transform tf_up;
    tf_up.setRotation(tf::Quaternion(q_up.x(), q_up.y(), q_up.z(), q_up.w()));
    Eigen::Vector3d e_up1, e_up2, e_up;
    tf_up.getBasis().getRPY(e_up1[2], e_up1[1], e_up1[0]);
    tf_up.getBasis().getRPY(e_up2[2], e_up2[1], e_up2[0], 2);
    //Use smaller rotation solution
    if (e_up1.norm() < e_up2.norm())
      e_up = e_up1;
    else
      e_up = e_up2;

    //Store External Estimate
    _externalEstimate.pos.x() = t_up[1];
    _externalEstimate.pos.y() = t_up[2];
    _externalEstimate.pos.z() = t_up[0];
    _externalEstimate.rot_x = Angle(e_up[1]);
    _externalEstimate.rot_y = Angle(e_up[0]);
    _externalEstimate.rot_z = Angle(e_up[2]);

    //Use External Estimate
    if (_isLaserOdometryDegenerate || _forceExternalPriorUseForMapping) {
      _transformTobeMapped.pos.x() = _externalEstimate.pos.x();
      _transformTobeMapped.pos.y() = _externalEstimate.pos.y();
      _transformTobeMapped.pos.z() = _externalEstimate.pos.z();
      _transformTobeMapped.rot_x = Angle(_externalEstimate.rot_x.rad());
      _transformTobeMapped.rot_y = Angle(_externalEstimate.rot_y.rad());
      _transformTobeMapped.rot_z = Angle(_externalEstimate.rot_z.rad());
    }

    //Use external frame for gravity alignment - Use as Prior
    if (_useExtRotationAlignment) {
      //Get Odom pose in LiDAR
      tf::StampedTransform tf_LO;
      static ros::Duration time_offset(_extOdomTimeOffset);
      ros::Time currPCL_ts = _currentPCLTime + time_offset;
      _tfListener.lookupTransform(_lidarFrame, _extFixedFrame, currPCL_ts, tf_LO);
      Eigen::Vector3d t_LO(tf_LO.getOrigin().x(), tf_LO.getOrigin().y(), tf_LO.getOrigin().z());
      Eigen::Quaterniond q_LO(tf_LO.getRotation().w(), tf_LO.getRotation().x(), tf_LO.getRotation().y(), tf_LO.getRotation().z());
      q_LO.normalize();
      gtsam::Pose3 T_LO = gtsam::Pose3(gtsam::Rot3(q_LO.toRotationMatrix()), t_LO);

      //Get Robot in Map
      Eigen::Vector3d t_ML(_transformTobeMapped.pos.z(), _transformTobeMapped.pos.x(), _transformTobeMapped.pos.y());
      Eigen::Quaterniond q_ML;
      q_ML = Eigen::AngleAxisd(_transformTobeMapped.rot_y.rad(), Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(_transformTobeMapped.rot_x.rad(), Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(_transformTobeMapped.rot_z.rad(), Eigen::Vector3d::UnitX());
      q_ML.normalize();
      gtsam::Pose3 T_ML = gtsam::Pose3(gtsam::Rot3(q_ML.toRotationMatrix()), t_ML);

      //Get transformation of Odometry frame in Map frame (capture relative drift)
      gtsam::Pose3 T_MO = T_ML * T_LO;

      //Estimate gravity vector in Map frame (Z-up)
      gtsam::Vector3 M_gravDir = T_MO.rotation().rotate(gtsam::Vector3::UnitZ()).normalized();

      //Estimate relative rotation between gravity vectors in Map and Odom frame(assumed to be perfectly aligned with gravity)
      gtsam::Rot3 r_ML_fix = gtsam::Rot3(gtsam::Quaternion().setFromTwoVectors(M_gravDir, gtsam::Vector3::UnitZ()).normalized());

      //Apply rotation fix to LiDAR-to-Map rotation
      gtsam::Rot3 r_ML_aligned = r_ML_fix * T_ML.rotation();
      
      //DEUBG
      // std::cout << "Gravity vector alignment - before: " << M_gravDir.transpose()
      //           << ", after: " << r_ML_aligned.rotate(T_LO.rotation().rotate(gtsam::Vector3::UnitZ())).normalized().transpose() << std::endl;
      // std::cout << "Gravity alignment RPY(deg) - before: " << _transformTobeMapped.rot_z.deg() << "," << _transformTobeMapped.rot_x.deg() << "," << _transformTobeMapped.rot_y.deg()
      //           << ", after: " << r_ML_aligned.rpy().transpose() * 180.0 / M_PI << std::endl;
      //DEUBG

      //Overwrite
      _transformTobeMapped.rot_x = Angle(r_ML_aligned.pitch());
      _transformTobeMapped.rot_y = Angle(r_ML_aligned.yaw());
      _transformTobeMapped.rot_z = Angle(r_ML_aligned.roll());
    }

    //DEBUG
    if (_verboseLevel > 1)
      std::cout << "\033[32mMAPPING\033[0m EXT PRIOR (x,y,z): [" << _externalEstimate.pos.z() << " " << _externalEstimate.pos.x() << " " << _externalEstimate.pos.y()
                << "], (r,p,y): [" << _externalEstimate.rot_z.deg() << " " << _externalEstimate.rot_x.deg() << " " << _externalEstimate.rot_y.deg()
                << "], Type: " << (primaryPrior ? "P" : "F")
                << ", Use: " << ((_isLaserOdometryDegenerate || _forceExternalPriorUseForMapping) ? "Yes" : "No") << std::endl;
    //DEBUG
  }
  //CUTOMIZATION
}

void BasicLaserMapping::pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po) {
  po.x = pi.x;
  po.y = pi.y;
  po.z = pi.z;
  po.intensity = pi.intensity;

  rotateZXY(po, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);

  po.x += _transformTobeMapped.pos.x();
  po.y += _transformTobeMapped.pos.y();
  po.z += _transformTobeMapped.pos.z();
}

void BasicLaserMapping::pointAssociateTobeMapped(const pcl::PointXYZI& pi, pcl::PointXYZI& po) {
  po.x = pi.x - _transformTobeMapped.pos.x();
  po.y = pi.y - _transformTobeMapped.pos.y();
  po.z = pi.z - _transformTobeMapped.pos.z();
  po.intensity = pi.intensity;

  rotateYXZ(po, -_transformTobeMapped.rot_y, -_transformTobeMapped.rot_x, -_transformTobeMapped.rot_z);
}

void BasicLaserMapping::transformFullResToMap() {
  // transform full resolution input cloud to map
  for (auto& pt : *_laserCloudFullRes)
    pointAssociateToMap(pt, pt);
}

bool BasicLaserMapping::createDownsizedMap() {
  // create new map cloud according to the input output ratio
  _mapFrameCount++;
  if (_mapFrameCount < _mapFrameNum)
    return false;

  _mapFrameCount = 0;

  // accumulate map cloud
  _laserCloudSurround->clear();
  for (auto ind : _laserCloudSurroundInd) {
    *_laserCloudSurround += *_laserCloudCornerArray[ind];
    *_laserCloudSurround += *_laserCloudSurfArray[ind];
  }

  // down size map cloud
  _laserCloudSurroundDS->clear();
  _downSizeFilterCorner.setInputCloud(_laserCloudSurround);
  _downSizeFilterCorner.filter(*_laserCloudSurroundDS);
  return true;
}

bool BasicLaserMapping::process(Time const& laserOdometryTime) {
  // Update frame count and time
  _frameCount++;
  _laserOdometryTime = laserOdometryTime;

  pcl::PointXYZI pointSel;

  // relate incoming data to map - update transform estimates
  transformAssociateToMap();

  //CUSTOMIZATION - Use saved submaps and initialization guess for map re-localziation
  if (_useSavedSubmapsForInit) {
    //LOAD Maps
    pcl::io::loadPCDFile((_saved_submaps_filepath + "savedCornerSubmap.pcd"), *_savedCornerSubmap);
    pcl::io::loadPCDFile((_saved_submaps_filepath + "savedSurfSubmap.pcd"), *_savedSurfSubmap);
    pcl::copyPointCloud(*_savedCornerSubmap, *_laserCloudCornerArray[_laserCloudNum / 2]);  //(21*11*21) / 2
    pcl::copyPointCloud(*_savedSurfSubmap, *_laserCloudSurfArray[_laserCloudNum / 2]);
    ROS_WARN_STREAM("LOADED SAVED CORNER/SURFACE SUBMAPS FOR INITIALIZATION WITH " << _laserCloudCornerArray[_laserCloudNum / 2]->points.size() << "/"
                                                                                   << _laserCloudSurfArray[_laserCloudNum / 2]->points.size() << " POINTS");
    //OVERWRITE Initial localization position and rotation if a guess is provided
    if (std::any_of(_W_submapLocGuess.begin(), _W_submapLocGuess.end(), [](float& f) { return f != 0; })) {
      _transformTobeMapped.pos.x() = _W_submapLocGuess[1];
      _transformTobeMapped.pos.y() = _W_submapLocGuess[2];
      _transformTobeMapped.pos.z() = _W_submapLocGuess[0];
      _transformTobeMapped.rot_x = _W_submapLocGuess[4];
      _transformTobeMapped.rot_y = _W_submapLocGuess[3];
      _transformTobeMapped.rot_z = _W_submapLocGuess[5];
      ROS_INFO_STREAM("SUBMAP Localization Guess (x,y,z): " << _W_submapLocGuess[0] << "," << _W_submapLocGuess[1] << "," << _W_submapLocGuess[2]
                                                            << ", (y,p,r): " << _W_submapLocGuess[3] << "," << _W_submapLocGuess[4] << "," << _W_submapLocGuess[5]);
    }
    _IncreOptIter = true;             //Increase allowed Iterations for convergence
    _useSavedSubmapsForInit = false;  //Load only once
  }
  //CUSTOMIZATION

  for (auto const& pt : _laserCloudCornerLast->points) {
    pointAssociateToMap(pt, pointSel);  //smk: getting SHARP points and transfroming them to map frame
    _laserCloudCornerStack->push_back(pointSel);
  }

  for (auto const& pt : _laserCloudSurfLast->points) {
    pointAssociateToMap(pt, pointSel);  //smk: getting FLAT points and transfroming them to map frame
    _laserCloudSurfStack->push_back(pointSel);
  }

  pcl::PointXYZI pointOnYAxis;  //smk:???
  pointOnYAxis.x = 0.0;
  pointOnYAxis.y = 10.0;
  pointOnYAxis.z = 0.0;
  pointAssociateToMap(pointOnYAxis, pointOnYAxis);

  static float const CUBE_SIZE = _mapCubeSize;
  static float const CUBE_HALF = CUBE_SIZE / 2.0;

  int centerCubeI = int((_transformTobeMapped.pos.x() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;  //smk:  ((x+25)/50)+10, if x=0 then int(10.5)=10
  int centerCubeJ = int((_transformTobeMapped.pos.y() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
  int centerCubeK = int((_transformTobeMapped.pos.z() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

  if (_transformTobeMapped.pos.x() + CUBE_HALF < 0)
    centerCubeI--;
  if (_transformTobeMapped.pos.y() + CUBE_HALF < 0)
    centerCubeJ--;
  if (_transformTobeMapped.pos.z() + CUBE_HALF < 0)
    centerCubeK--;

  while (centerCubeI < _neighborSubmapCubes) {
    for (int j = 0; j < _laserCloudHeight; j++) {
      for (int k = 0; k < _laserCloudDepth; k++) {
        for (int i = _laserCloudWidth - 1; i >= 1; i--) {
          const size_t indexA = toIndex(i, j, k);
          const size_t indexB = toIndex(i - 1, j, k);
          std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);  //smk: seems like we are reversing the pointcloud
          std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
        }
        const size_t indexC = toIndex(0, j, k); //smk: fix from github.com/laboshinl/loam_velodyne/pull/110
        _laserCloudCornerArray[indexC]->clear();
        _laserCloudSurfArray[indexC]->clear();
      }
    }
    centerCubeI++;
    _laserCloudCenWidth++;
  }

  while (centerCubeI >= _laserCloudWidth - _neighborSubmapCubes) {
    for (int j = 0; j < _laserCloudHeight; j++) {
      for (int k = 0; k < _laserCloudDepth; k++) {
        for (int i = 0; i < _laserCloudWidth - 1; i++) {
          const size_t indexA = toIndex(i, j, k);
          const size_t indexB = toIndex(i + 1, j, k);
          std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
          std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
        }
        const size_t indexC = toIndex(_laserCloudWidth - 1, j, k);
        _laserCloudCornerArray[indexC]->clear();
        _laserCloudSurfArray[indexC]->clear();
      }
    }
    centerCubeI--;
    _laserCloudCenWidth--;
  }

  while (centerCubeJ < _neighborSubmapCubes) {
    for (int i = 0; i < _laserCloudWidth; i++) {
      for (int k = 0; k < _laserCloudDepth; k++) {
        for (int j = _laserCloudHeight - 1; j >= 1; j--) {
          const size_t indexA = toIndex(i, j, k);
          const size_t indexB = toIndex(i, j - 1, k);
          std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
          std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
        }
        const size_t indexC = toIndex(i, 0, k);
        _laserCloudCornerArray[indexC]->clear();
        _laserCloudSurfArray[indexC]->clear();
      }
    }
    centerCubeJ++;
    _laserCloudCenHeight++;
  }

  while (centerCubeJ >= _laserCloudHeight - _neighborSubmapCubes) {
    for (int i = 0; i < _laserCloudWidth; i++) {
      for (int k = 0; k < _laserCloudDepth; k++) {
        for (int j = 0; j < _laserCloudHeight - 1; j++) {
          const size_t indexA = toIndex(i, j, k);
          const size_t indexB = toIndex(i, j + 1, k);
          std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
          std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
        }
        const size_t indexC = toIndex(i, _laserCloudHeight - 1, k);
        _laserCloudCornerArray[indexC]->clear();
        _laserCloudSurfArray[indexC]->clear();
      }
    }
    centerCubeJ--;
    _laserCloudCenHeight--;
  }

  while (centerCubeK < _neighborSubmapCubes) {
    for (int i = 0; i < _laserCloudWidth; i++) {
      for (int j = 0; j < _laserCloudHeight; j++) {
        for (int k = _laserCloudDepth - 1; k >= 1; k--) {
          const size_t indexA = toIndex(i, j, k);
          const size_t indexB = toIndex(i, j, k - 1);
          std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
          std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
        }
        const size_t indexC = toIndex(i, j, 0);
        _laserCloudCornerArray[indexC]->clear();
        _laserCloudSurfArray[indexC]->clear();
      }
    }
    centerCubeK++;
    _laserCloudCenDepth++;
  }

  while (centerCubeK >= _laserCloudDepth - _neighborSubmapCubes) {
    for (int i = 0; i < _laserCloudWidth; i++) {
      for (int j = 0; j < _laserCloudHeight; j++) {
        for (int k = 0; k < _laserCloudDepth - 1; k++) {
          const size_t indexA = toIndex(i, j, k);
          const size_t indexB = toIndex(i, j, k + 1);
          std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
          std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
        }
        const size_t indexC = toIndex(i, j, _laserCloudDepth - 1);
        _laserCloudCornerArray[indexC]->clear();
        _laserCloudSurfArray[indexC]->clear();
      }
    }
    centerCubeK--;
    _laserCloudCenDepth--;
  }

  _laserCloudValidInd.clear();
  _laserCloudSurroundInd.clear();
  for (int i = centerCubeI - _neighborSubmapCubes; i <= centerCubeI + _neighborSubmapCubes; ++i) {
    for (int j = centerCubeJ - _neighborSubmapCubes; j <= centerCubeJ + _neighborSubmapCubes; ++j) {
      for (int k = centerCubeK - _neighborSubmapCubes; k <= centerCubeK + _neighborSubmapCubes; ++k) {
        if (i >= 0 && i < _laserCloudWidth &&
            j >= 0 && j < _laserCloudHeight &&
            k >= 0 && k < _laserCloudDepth) {
          float centerX = CUBE_SIZE * (i - _laserCloudCenWidth);   //smk: 50 * (i - 10) // so if starting cube CenterX=0 //center of starting cube is 0 next cube center is 50
          float centerY = CUBE_SIZE * (j - _laserCloudCenHeight);  //smk: this is expresing center of current cube in meters
          float centerZ = CUBE_SIZE * (k - _laserCloudCenDepth);

          pcl::PointXYZI transform_pos = (pcl::PointXYZI)_transformTobeMapped.pos;

          bool isInLaserFOV = false;
          for (int ii = -1; ii <= 1; ii += 2) {
            for (int jj = -1; jj <= 1; jj += 2) {
              for (int kk = -1; kk <= 1; kk += 2) {
                pcl::PointXYZI corner;
                corner.x = centerX + CUBE_HALF * ii;  //smk: centerX=0 for start cube so corner.x will be +/- 25 i.e. boundary of cube infront and behind the cube
                corner.y = centerY + CUBE_HALF * jj;  //smk: expressing boundries of the cube in meters
                corner.z = centerZ + CUBE_HALF * kk;

                float squaredSide1 = calcSquaredDiff(transform_pos, corner);  //smk: squared distance from all three boundaries
                float squaredSide2 = calcSquaredDiff(pointOnYAxis, corner);   //smk: this is a point at the current robot position but offset by 10 meters in height

                float check1 = 100.0f + squaredSide1 - squaredSide2 - 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                float check2 = 100.0f + squaredSide1 - squaredSide2 + 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                if (check1 < 0 && check2 > 0) {
                  isInLaserFOV = true;
                }
              }
            }
          }

          size_t cubeIdx = i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k;
          if (isInLaserFOV) {
            _laserCloudValidInd.push_back(cubeIdx);
          }
          _laserCloudSurroundInd.push_back(cubeIdx);
        }
      }
    }
  }

  // prepare valid map corner and surface cloud for pose optimization
  _laserCloudCornerFromMap->clear();
  _laserCloudSurfFromMap->clear();
  for (auto const& ind : _laserCloudValidInd) {
    *_laserCloudCornerFromMap += *_laserCloudCornerArray[ind];
    *_laserCloudSurfFromMap += *_laserCloudSurfArray[ind];
  }

  // prepare feature stack clouds for pose optimization
  for (auto& pt : *_laserCloudCornerStack)
    pointAssociateTobeMapped(pt, pt);

  for (auto& pt : *_laserCloudSurfStack)
    pointAssociateTobeMapped(pt, pt);

  // down sample feature stack clouds
  _laserCloudCornerStackDS->clear();
  _downSizeFilterCorner.setInputCloud(_laserCloudCornerStack);
  _downSizeFilterCorner.filter(*_laserCloudCornerStackDS);
  size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->size();

  _laserCloudSurfStackDS->clear();
  _downSizeFilterSurf.setInputCloud(_laserCloudSurfStack);
  _downSizeFilterSurf.filter(*_laserCloudSurfStackDS);
  size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->size();

  _laserCloudCornerStack->clear();
  _laserCloudSurfStack->clear();

  // run pose optimization
  optimizeTransformTobeMapped();

  // store down sized corner stack points in corresponding cube clouds
  for (int i = 0; i < laserCloudCornerStackNum; i++) {
    pointAssociateToMap(_laserCloudCornerStackDS->points[i], pointSel);

    int cubeI = int((pointSel.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
    int cubeJ = int((pointSel.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
    int cubeK = int((pointSel.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

    if (pointSel.x + CUBE_HALF < 0)
      cubeI--;
    if (pointSel.y + CUBE_HALF < 0)
      cubeJ--;
    if (pointSel.z + CUBE_HALF < 0)
      cubeK--;

    if (cubeI >= 0 && cubeI < _laserCloudWidth &&
        cubeJ >= 0 && cubeJ < _laserCloudHeight &&
        cubeK >= 0 && cubeK < _laserCloudDepth) {
      size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
      _laserCloudCornerArray[cubeInd]->push_back(pointSel);
    }
  }

  // store down sized surface stack points in corresponding cube clouds
  for (int i = 0; i < laserCloudSurfStackNum; i++) {
    pointAssociateToMap(_laserCloudSurfStackDS->points[i], pointSel);

    int cubeI = int((pointSel.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
    int cubeJ = int((pointSel.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
    int cubeK = int((pointSel.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

    if (pointSel.x + CUBE_HALF < 0)
      cubeI--;
    if (pointSel.y + CUBE_HALF < 0)
      cubeJ--;
    if (pointSel.z + CUBE_HALF < 0)
      cubeK--;

    if (cubeI >= 0 && cubeI < _laserCloudWidth &&
        cubeJ >= 0 && cubeJ < _laserCloudHeight &&
        cubeK >= 0 && cubeK < _laserCloudDepth) {
      size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
      _laserCloudSurfArray[cubeInd]->push_back(pointSel);
    }
  }

  // down size all valid (within field of view) feature cube clouds
  for (auto const& ind : _laserCloudValidInd) {
    _laserCloudCornerDSArray[ind]->clear();
    _downSizeFilterCorner.setInputCloud(_laserCloudCornerArray[ind]);
    _downSizeFilterCorner.filter(*_laserCloudCornerDSArray[ind]);

    _laserCloudSurfDSArray[ind]->clear();
    _downSizeFilterSurf.setInputCloud(_laserCloudSurfArray[ind]);
    _downSizeFilterSurf.filter(*_laserCloudSurfDSArray[ind]);

    // swap cube clouds for next processing
    _laserCloudCornerArray[ind].swap(_laserCloudCornerDSArray[ind]);
    _laserCloudSurfArray[ind].swap(_laserCloudSurfDSArray[ind]);

    //Add valid cloud indices to occupied map cube list
    _occupiedCubeInd.insert(ind);
  }

  return true;
}

void BasicLaserMapping::updateIMU(IMUState2 const& newState) {
  _imuHistory.push(newState);
}

void BasicLaserMapping::updateOdometry(double pitch, double yaw, double roll, double x, double y, double z) {
  _transformSum.rot_x = pitch;
  _transformSum.rot_y = yaw;
  _transformSum.rot_z = roll;

  _transformSum.pos.x() = float(x);
  _transformSum.pos.y() = float(y);
  _transformSum.pos.z() = float(z);
}

void BasicLaserMapping::updateOdometry(Twist const& twist) {
  _transformSum = twist;
}

nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeCornerFromMap;
nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeSurfFromMap;

void BasicLaserMapping::optimizeTransformTobeMapped() {
  if (_laserCloudCornerFromMap->size() <= 10 || _laserCloudSurfFromMap->size() <= 100)
    return;

  //CUSTOMIZATION - Increase allowed Iterations for convergence
  static size_t tmpMaxIterSave = _maxIterations;
  if (_IncreOptIter) {
    _maxIterations = 3 * tmpMaxIterSave;  //3x _maxIterations
    ROS_INFO_STREAM_ONCE("Allowing more iterations for initial convergence, _maxIterations:" << _maxIterations);
  } else {
    _maxIterations = tmpMaxIterSave;
    ROS_INFO_STREAM_ONCE("Reseting number of iterations to original, _maxIterations:" << _maxIterations);
  }
  //CUSTOMIZATION

  pcl::PointXYZI pointSel, pointOri, /*pointProj, */ coeff;

  std::vector<int> pointSearchInd(5, 0);
  std::vector<float> pointSearchSqDis(5, 0);

  kdtreeCornerFromMap.setInputCloud(_laserCloudCornerFromMap);
  kdtreeSurfFromMap.setInputCloud(_laserCloudSurfFromMap);

  Eigen::Matrix<float, 5, 3> matA0;
  Eigen::Matrix<float, 5, 1> matB0;
  Eigen::Vector3f matX0;
  Eigen::Matrix3f matA1;
  Eigen::Matrix<float, 1, 3> matD1;
  Eigen::Matrix3f matV1;

  matA0.setZero();
  matB0.setConstant(-1);
  matX0.setZero();

  matA1.setZero();
  matD1.setZero();
  matV1.setZero();

  Eigen::Matrix<float, 6, 6> matP;

  size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->size();
  size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->size();

  double deltaT = std::numeric_limits<double>::quiet_NaN();
  double deltaR = std::numeric_limits<double>::quiet_NaN();
  bool isDegenerate = false;
  bool error_converged = false;
  size_t iterCount = 0;
  std::clock_t time_start = std::clock();
  //Optimization loop
  for (; iterCount < _maxIterations; ++iterCount) {
    _laserCloudOri.clear();
    _coeffSel.clear();

    //Map associate Corner points
    for (int i = 0; i < laserCloudCornerStackNum; i++) {
      pointOri = _laserCloudCornerStackDS->points[i];
      pointAssociateToMap(pointOri, pointSel);
      kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);  //smk: find 5 closest points in KDTree

      if (pointSearchSqDis[4] < 1.0) {
        Vector3 vc(0, 0, 0);  //smk: if all nearest points are within 1.0 squared distance calculate centeroid of the cluster

        for (int j = 0; j < 5; j++)
          vc += Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]);
        vc /= 5.0;

        Eigen::Matrix3f mat_a;  //smk: this is covariance matrix of the nearest point cluster to the current point
        mat_a.setZero();

        for (int j = 0; j < 5; j++) {
          Vector3 a = Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]) - vc;

          mat_a(0, 0) += a.x() * a.x();  //smk: https://github.com/laboshinl/loam/commit/b5b326ede5757995cc1583cde51b92e0e2d07796
          mat_a(1, 0) += a.x() * a.y();  //smk: swap upper and lower matrices
          mat_a(2, 0) += a.x() * a.z();  //smk: SelfAdjointEigenSolver only references lower part of the matrix
          mat_a(1, 1) += a.y() * a.y();
          mat_a(2, 1) += a.y() * a.z();
          mat_a(2, 2) += a.z() * a.z();
        }
        matA1 = mat_a / 5.0;  //smk: covariance matrix

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);
        matD1 = esolver.eigenvalues().real();  //smk: 1x3 vector, arranged in ascending order
        matV1 = esolver.eigenvectors().real();

        if (matD1(0, 2) > 3 * matD1(0, 1))  //smk: largest eigen is atleast 3 times larger than the second to be considered an edge
        {
          float x0 = pointSel.x;
          float y0 = pointSel.y;
          float z0 = pointSel.z;
          float x1 = vc.x() + 0.1 * matV1(0, 2);
          float y1 = vc.y() + 0.1 * matV1(1, 2);
          float z1 = vc.z() + 0.1 * matV1(2, 2);
          float x2 = vc.x() - 0.1 * matV1(0, 2);
          float y2 = vc.y() - 0.1 * matV1(1, 2);
          float z2 = vc.z() - 0.1 * matV1(2, 2);

          float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

          float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

          float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

          float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

          float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

          float ld2 = a012 / l12;

          //                // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
          //                pointProj = pointSel;
          //                pointProj.x -= la * ld2;
          //                pointProj.y -= lb * ld2;
          //                pointProj.z -= lc * ld2;

          float s = 1 - 0.9f * fabs(ld2);

          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

          if (s > 0.1) {
            _laserCloudOri.push_back(pointOri);
            _coeffSel.push_back(coeff);
          }
        }
      }
    }

    //Map associate Surface points
    for (int i = 0; i < laserCloudSurfStackNum; i++) {
      pointOri = _laserCloudSurfStackDS->points[i];
      pointAssociateToMap(pointOri, pointSel);
      kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

      if (pointSearchSqDis[4] < 1.0) {
        for (int j = 0; j < 5; j++) {
          matA0(j, 0) = _laserCloudSurfFromMap->points[pointSearchInd[j]].x;
          matA0(j, 1) = _laserCloudSurfFromMap->points[pointSearchInd[j]].y;
          matA0(j, 2) = _laserCloudSurfFromMap->points[pointSearchInd[j]].z;
        }
        matX0 = matA0.colPivHouseholderQr().solve(matB0);

        float pa = matX0(0, 0);
        float pb = matX0(1, 0);
        float pc = matX0(2, 0);
        float pd = 1;

        float ps = sqrt(pa * pa + pb * pb + pc * pc);
        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
          if (fabs(pa * _laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                   pb * _laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                   pc * _laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2) {
            planeValid = false;
            break;
          }
        }

        if (planeValid) {
          float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

          //                // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
          //                pointProj = pointSel;
          //                pointProj.x -= pa * pd2;
          //                pointProj.y -= pb * pd2;
          //                pointProj.z -= pc * pd2;

          float s = 1 - 0.9f * fabs(pd2) / sqrt(calcPointDistance(pointSel));

          coeff.x = s * pa;
          coeff.y = s * pb;
          coeff.z = s * pc;
          coeff.intensity = s * pd2;

          if (s > 0.1) {
            _laserCloudOri.push_back(pointOri);
            _coeffSel.push_back(coeff);
          }
        }
      }
    }

    float srx = _transformTobeMapped.rot_x.sin();
    float crx = _transformTobeMapped.rot_x.cos();
    float sry = _transformTobeMapped.rot_y.sin();
    float cry = _transformTobeMapped.rot_y.cos();
    float srz = _transformTobeMapped.rot_z.sin();
    float crz = _transformTobeMapped.rot_z.cos();

    size_t laserCloudSelNum = _laserCloudOri.size();
    if (laserCloudSelNum < 50)
      continue;

    Eigen::Matrix<float, Eigen::Dynamic, 6> matA(laserCloudSelNum, 6);
    Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, laserCloudSelNum);
    Eigen::Matrix<float, 6, 6> matAtA;
    Eigen::VectorXf matB(laserCloudSelNum);
    Eigen::VectorXf matAtB;
    Eigen::VectorXf matX;

    for (int i = 0; i < laserCloudSelNum; i++) {
      pointOri = _laserCloudOri.points[i];
      coeff = _coeffSel.points[i];

      float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

      float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

      float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;

      matA(i, 0) = arx;
      matA(i, 1) = ary;
      matA(i, 2) = arz;
      matA(i, 3) = coeff.x;
      matA(i, 4) = coeff.y;
      matA(i, 5) = coeff.z;
      matB(i, 0) = -coeff.intensity;
    }

    matAt = matA.transpose();
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    matX = matAtA.colPivHouseholderQr().solve(matAtB);  //ORIGINAL smk: now matX is 6x1

    //smk: Check if problem is degenerate from beginning
    if (iterCount == 0) {
      Eigen::Matrix<float, 1, 6> matE;
      Eigen::Matrix<float, 6, 6> matV;
      Eigen::Matrix<float, 6, 6> matV2;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> esolver(matAtA);
      matE = esolver.eigenvalues().real();
      matV = esolver.eigenvectors().real();

      matV2 = matV;

      isDegenerate = false;
      float eignThre[6] = {_mapDegenEigVal, _mapDegenEigVal, _mapDegenEigVal, _mapDegenEigVal, _mapDegenEigVal, _mapDegenEigVal};

      for (int i = 0; i < 6; i++) {
        if (matE(0, i) < eignThre[i]) {
          for (int j = 0; j < 6; j++) {
            matV2(i, j) = 0;
          }
          isDegenerate = true;
        } else {
          break;
        }
      }
      matP = matV.inverse() * matV2;

      //CUSTOMIZATION
      //Enable External Estimate passthrough if solution was degenerate
      if (isDegenerate && _isExternalPriorHealthy) {
        ROS_WARN_THROTTLE(0.5, "*** LOAM  - MAPPING Degenerate - Using External Estimate ***");
        _transformTobeMapped.pos.x() = _externalEstimate.pos.x();
        _transformTobeMapped.pos.y() = _externalEstimate.pos.y();
        _transformTobeMapped.pos.z() = _externalEstimate.pos.z();
        _transformTobeMapped.rot_x = Angle(_externalEstimate.rot_x.rad());
        _transformTobeMapped.rot_y = Angle(_externalEstimate.rot_y.rad());
        _transformTobeMapped.rot_z = Angle(_externalEstimate.rot_z.rad());
        break;
      } else if (isDegenerate)
        ROS_WARN_THROTTLE(0.5, "*** LOAM  - MAPPING Degenerate ***");
      //CUSTOMIZATION
    }

    if (isDegenerate) {
      Eigen::Matrix<float, 6, 1> matX2(matX);
      matX = matP * matX2;
    }

    _transformTobeMapped.rot_x += matX(0, 0);
    _transformTobeMapped.rot_y += matX(1, 0);
    _transformTobeMapped.rot_z += matX(2, 0);
    _transformTobeMapped.pos.x() += matX(3, 0);
    _transformTobeMapped.pos.y() += matX(4, 0);
    _transformTobeMapped.pos.z() += matX(5, 0);

    deltaR = std::sqrt(std::pow(rad2deg(matX(0, 0)), 2) +
                       std::pow(rad2deg(matX(1, 0)), 2) +
                       std::pow(rad2deg(matX(2, 0)), 2));
    deltaT = std::sqrt(std::pow(matX(3, 0) * 100, 2) +
                       std::pow(matX(4, 0) * 100, 2) +
                       std::pow(matX(5, 0) * 100, 2));
    if (deltaR < _deltaRAbort && deltaT < _deltaTAbort && iterCount > _minIterations) {
      //CUSTOMIZATION
      //Calculation of the vairance of the solution
      // Eigen::VectorXf residual = (matA * matX) - matB;
      // float mse = static_cast<float>(residual.transpose() * residual) / (laserCloudSelNum - 6); //smk: matlab nlinfit
      // float variance = residual.norm() / (laserCloudSelNum - 6);
      // Eigen::Matrix<float, 6, 6> covMat = mse * matAtA.inverse();
      // std::cout << "Mapping, MSE: " << mse << ", covMat diagonal: " << covMat.diagonal().transpose() << std::endl;
      //CUSTOMIZATION
      error_converged = true;
      break;
    }
  }

  //DEBUG - Optimization Status
  auto t = static_cast<double>(std::clock() - time_start) / CLOCKS_PER_SEC * 1000.0;
  if (_verboseLevel > 0) {
    std::cout << "\033[32mMAPPING:\033[0m - Converged:" << (error_converged ? " \033[32mTrue\033[0m" : "\033[31mFalse\033[0m")
              << ", Degenerate:" << (isDegenerate ? " \033[31mTrue\033[0m" : "\033[32mFalse\033[0m")
              << ", Iter:" << iterCount + 1
              << ", Residual(T/R):" << deltaT << "/" << deltaR
              << ", Time(ms):" << t
              << (t < 175 ? " \033[32mGOOD\033[0m" : " \033[31mSLOW\033[0m") << "\n";
  }
  // _mappingLogFile << t << "\n";
  //DEBUG - Optimization Status

  //Update Optimization Status Message
  _optStatusMsg.converged = error_converged;
  _optStatusMsg.translation_residual = deltaT;
  _optStatusMsg.rotation_residual = deltaR;
  _optStatusMsg.degenerate = isDegenerate;
  _optStatusMsg.iteration_count = iterCount + 1;
  _optStatusMsg.iteration_max = _maxIterations;
  _optStatusMsg.optimization_time = static_cast<double>(std::clock() - time_start) / CLOCKS_PER_SEC;
  //Status
  static const double optTimeLimit = 0.2;  //5Hz
  if (_optStatusMsg.converged) {
    if (_optStatusMsg.optimization_time < optTimeLimit)
      _optStatusMsg.status = "CONVERGED";
    else
      _optStatusMsg.status = "SLOW_CONVERGENCE";
  } else {
    if (isDegenerate)
      _optStatusMsg.status = "DEGENERATE";
    else
      _optStatusMsg.status = "CONVERGENCE_FAILURE";
  }

  //CUSTOMIZATION
  if (iterCount > 1)
    _IncreOptIter = false;
  //CUSTOMIZATION

  //DEBUG
  if (_verboseLevel > 2) {
    if (iterCount == _maxIterations - 1)
      std::cout << "\033[32mMAPPING:\033[0m - \033[31mMAX_ITERATIONS\033[0m, Iter:" << iterCount
                << ", (dT/dR) FINAL: " << deltaT << "/" << deltaR
                << ", time(ms): " << _optStatusMsg.optimization_time * 1000.0 << std::endl;
    else if (_optStatusMsg.optimization_time > optTimeLimit)
      std::cout << "\033[32mMAPPING:\033[0m - \033[33mSLOW_CONVERGENCE\033[0m, Iter:" << iterCount
                << ", (dT/dR) FINAL: " << deltaT << "/" << deltaR
                << ", time(ms): " << _optStatusMsg.optimization_time * 1000.0 << std::endl;
    else
      std::cout << "\033[32mMAPPING:\033[0m - \033[32mCONVERGED\033[0m, Iter:" << iterCount
                << ", (dT/dR) FINAL: " << deltaT << "/" << deltaR
                << ", time(ms): " << _optStatusMsg.optimization_time * 1000.0 << std::endl;
  }
  //DEBUG

  transformUpdate();

  //DEBUG
  if (_verboseLevel > 1)
    std::cout << "\033[32mMAPPING\033[0m RESULT (x,y,z): [" << _transformAftMapped.pos.z() << " " << _transformAftMapped.pos.x() << " " << _transformAftMapped.pos.y()
              << "], (r,p,y): [" << _transformAftMapped.rot_z.deg() << " " << _transformAftMapped.rot_x.deg() << " " << _transformAftMapped.rot_y.deg() << "]\n";
  //DEBUG
}

void BasicLaserMapping::transformUpdate()  //smk: this just updates roll and pitch in LiDAR frame with values from IMU in a weighted sum manner
{
  if (0 < _imuHistory.size()) {
    size_t imuIdx = 0;

    while (imuIdx < _imuHistory.size() - 1 && toSec(_laserOdometryTime - _imuHistory[imuIdx].stamp) + _scanPeriod > 0) {
      imuIdx++;
    }

    IMUState2 imuCur;

    if (imuIdx == 0 || toSec(_laserOdometryTime - _imuHistory[imuIdx].stamp) + _scanPeriod > 0) {
      // scan time newer then newest or older than oldest IMU message
      imuCur = _imuHistory[imuIdx];
    } else {
      float ratio = (toSec(_imuHistory[imuIdx].stamp - _laserOdometryTime) - _scanPeriod) / toSec(_imuHistory[imuIdx].stamp - _imuHistory[imuIdx - 1].stamp);

      IMUState2::interpolate(_imuHistory[imuIdx], _imuHistory[imuIdx - 1], ratio, imuCur);
    }

    _transformTobeMapped.rot_x = 0.998 * _transformTobeMapped.rot_x.rad() + 0.002 * imuCur.pitch.rad();
    _transformTobeMapped.rot_z = 0.998 * _transformTobeMapped.rot_z.rad() + 0.002 * imuCur.roll.rad();
  }

  _transformBefMapped = _transformSum;
  _transformAftMapped = _transformTobeMapped;
}

//CUSTOMIZATION
void BasicLaserMapping::updatePCLTime(const ros::Time& currentTimeStamp) {
  _lastPCLTime = _currentPCLTime;
  _currentPCLTime = currentTimeStamp;
  //DEBUG
  // std::cout << "\033[32mMAPPING\033[0m _lastPCLTime: " << _lastPCLTime.sec << "," << _lastPCLTime.nsec
  //           << ", _currentPCLTime: " << _currentPCLTime.sec << "," << _currentPCLTime.nsec
  //           << ", diff: " << (_currentPCLTime - _lastPCLTime).toSec() << std::endl;
  //DEBUG
}

bool BasicLaserMapping::calcExternalPrior(Eigen::Vector3d& transPriorXYZ, Eigen::Vector3d& rotPriorXYZ, bool useFallback /*= false*/) {
  transPriorXYZ.setZero();
  rotPriorXYZ.setZero();
  tf::StampedTransform T_S2S1;  //Relative Transform of S1 w.r.t S2 using fixed frame
  tf::StampedTransform T_LS;    //Transform from external Sensor to LiDAR Frame - Extrinsic Calibration
  static ros::Duration time_offset(_extOdomTimeOffset);
  static ros::Duration fallback_time_offset(_fallbackExtOdomTimeOffset);

  //Skip first two pointclouds from external prior lookup as _transformAftMapped has not updated yet
  if (_frameCount < 3) {
    std::cout << "\033[32mMAPPING\033[0m External Prior - Skip for PCL: " << _frameCount << std::endl;
    return false;
  }

  if (!useFallback) {
    try {
      //Relative Transform
      ros::Time currPCL_ts = _currentPCLTime + time_offset;
      ros::Time lastPCL_ts = _lastPCLTime + time_offset;
      _tfListener.lookupTransform(_extOdomFrame, currPCL_ts,
                                  _extOdomFrame, lastPCL_ts,
                                  _extFixedFrame, T_S2S1);
      //External Sensor-to-LiDAR transform
      _tfListener.lookupTransform(_lidarFrame, _extSensorFrame, ros::Time(0), T_LS);
    } catch (tf::TransformException ex) {
      ROS_WARN_THROTTLE(5.0, "%s", ex.what());
      return false;
    }
  } else {
    try {
      //Relative Transform
      ros::Time currPCL_ts = _currentPCLTime + fallback_time_offset;
      ros::Time lastPCL_ts = _lastPCLTime + fallback_time_offset;
      _tfListener.lookupTransform(_fallbackExtOdomFrame, currPCL_ts,
                                  _fallbackExtOdomFrame, lastPCL_ts,
                                  _fallbackExtFixedFrame, T_S2S1);
      //External Sensor-to-LiDAR transform
      _tfListener.lookupTransform(_lidarFrame, _fallbackExtSensorFrame,
                                  ros::Time(0), T_LS);
    } catch (tf::TransformException ex) {
      ROS_WARN_THROTTLE(5.0, "%s", ex.what());
      return false;
    }
  }

  // Calculate Prior in LiDAR or input Cloud Frame
  // The two frames can differ if LiDAR is mounted sideways but input cloud is rotated to give output aligned with ROS frame.
  // Assumption final frame either LiDAR or input Cloud frame follows ROS convention - This is important to resolve small euler angle increments from quaternion in near gimbal lock conditions(gimbal resolve needs to be tested in all possible orientations)
  tf::Transform prior;
  if (_isInputPCLRotated)
    prior = _Rot_inCloudLiDAR * T_LS * T_S2S1 * T_LS.inverse() * _Rot_inCloudLiDAR.inverse();
  else
    prior = T_LS * T_S2S1 * T_LS.inverse();

  //Prior Translation - Swap axis to align with LOAM frame
  transPriorXYZ << prior.getOrigin().y(), prior.getOrigin().z(), prior.getOrigin().x();

  //Prior Rotation
  Eigen::Vector3d e1, e2;
  prior.getBasis().getRPY(e1[2], e1[1], e1[0]);
  prior.getBasis().getRPY(e2[2], e2[1], e2[0], 2);
  //Set smaller rotation solution
  if (e1.norm() < e2.norm())
    rotPriorXYZ << e1[1], e1[0], e1[2];
  else
    rotPriorXYZ << e2[1], e2[0], e2[2];

  return true;
}

pcl::PointCloud<pcl::PointXYZI>& BasicLaserMapping::laserMap() {
  // Accumulate Surface Map
  pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (const auto& idx : _occupiedCubeInd)
    *mapCloud += *_laserCloudSurfArray[idx];

  //Downsample map cloud
  // auto t_map = std::clock();
  _laserMap->clear();
  _downSizeFilterMapVis.setInputCloud(mapCloud);
  _downSizeFilterMapVis.filter(*_laserMap);
  // auto t_map_ms = static_cast<double>(std::clock() - t_map) / CLOCKS_PER_SEC * 1000.0;
  // std::cout << "\033[32mMAPPING\033[0m Map visualization points, Org:" << mapCloud->size() << ", Downsampled: " << _laserMap->size() << ", time(ms): " << t_map_ms << std::endl;

  return *_laserMap;
}

pcl::PointCloud<pcl::PointXYZI>& BasicLaserMapping::getSubmapSearchCloud() {
  //Accumulated corner and surface search clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZI>());
  *currentCloud += *_laserCloudCornerStackDS;
  *currentCloud += *_laserCloudSurfStackDS;

  //Transform into map frame
  _submapSearchCloud->clear();
  _submapSearchCloud->reserve(currentCloud->size());
  pcl::PointXYZI tfPt;
  for (const auto& pt : currentCloud->points) {
    pointAssociateToMap(pt, tfPt);
    _submapSearchCloud->push_back(tfPt);
  }

  return *_submapSearchCloud;
}

//CUSTOMIZATION

}  // end namespace loam
