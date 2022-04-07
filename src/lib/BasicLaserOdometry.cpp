#include "loam/BasicLaserOdometry.h"

#include <pcl/filters/filter.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace loam {

using std::asin;
using std::atan2;
using std::cos;
using std::fabs;
using std::pow;
using std::sin;
using std::sqrt;

BasicLaserOdometry::BasicLaserOdometry(float scanPeriod, size_t maxIterations)
    : _scanPeriod(scanPeriod),
      _systemInited(false),
      _frameCount(0),
      _maxIterations(maxIterations),
      _minIterations(1),
      _deltaTAbort(0.1),
      _deltaRAbort(0.1),
      _odomDegenEigVal(10),
      _odomRelativeTranslationMax(0.15),
      _odomRelativeRotationMax(0.05),
      _cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()),
      _cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()),
      _surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>()),
      _surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
      _lastCornerCloud(new pcl::PointCloud<pcl::PointXYZI>()),
      _lastSurfaceCloud(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>()),
      _coeffSel(new pcl::PointCloud<pcl::PointXYZI>()) {
  //Initialize factor graph parameter object
  _graphMgr._params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
  // //Logging
  // _odometryLogFile.open("/home/shehryar/compslam_ws/src/compslam/odomEigen.csv", std::ios::out);
}

void BasicLaserOdometry::transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po) {
  float s;
  if (_undistortInputCloud)
    s = (1.f / _scanPeriod) * (pi.intensity - int(pi.intensity));
  else
    s = 1.0;

  po.x = pi.x - s * _transform.pos.x();
  po.y = pi.y - s * _transform.pos.y();
  po.z = pi.z - s * _transform.pos.z();
  po.intensity = pi.intensity;

  Angle rx = -s * _transform.rot_x.rad();
  Angle ry = -s * _transform.rot_y.rad();
  Angle rz = -s * _transform.rot_z.rad();
  rotateZXY(po, rz, rx, ry);
}

size_t BasicLaserOdometry::transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  size_t cloudSize = cloud->points.size();
  float s;

  for (size_t i = 0; i < cloudSize; i++) {
    pcl::PointXYZI& point = cloud->points[i];

    //Project current point to the end of the scan
    if (_undistortInputCloud)
      s = (1.f / _scanPeriod) * (point.intensity - int(point.intensity));
    else
      s = 1.0;

    point.x -= s * _transform.pos.x();
    point.y -= s * _transform.pos.y();
    point.z -= s * _transform.pos.z();
    point.intensity = int(point.intensity);

    Angle rx = -s * _transform.rot_x.rad();
    Angle ry = -s * _transform.rot_y.rad();
    Angle rz = -s * _transform.rot_z.rad();
    rotateZXY(point, rz, rx, ry);
    rotateYXZ(point, _transform.rot_y, _transform.rot_x, _transform.rot_z);

    point.x += _transform.pos.x() - _imuShiftFromStart.x();
    point.y += _transform.pos.y() - _imuShiftFromStart.y();
    point.z += _transform.pos.z() - _imuShiftFromStart.z();

    rotateZXY(point, _imuRollStart, _imuPitchStart, _imuYawStart);
    rotateYXZ(point, -_imuYawEnd, -_imuPitchEnd, -_imuRollEnd);
  }

  return cloudSize;
}

void BasicLaserOdometry::pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                                           const Angle& blx, const Angle& bly, const Angle& blz,
                                           const Angle& alx, const Angle& aly, const Angle& alz,
                                           Angle& acx, Angle& acy, Angle& acz) {
  float sbcx = bcx.sin();
  float cbcx = bcx.cos();
  float sbcy = bcy.sin();
  float cbcy = bcy.cos();
  float sbcz = bcz.sin();
  float cbcz = bcz.cos();

  float sblx = blx.sin();
  float cblx = blx.cos();
  float sbly = bly.sin();
  float cbly = bly.cos();
  float sblz = blz.sin();
  float cblz = blz.cos();

  float salx = alx.sin();
  float calx = alx.cos();
  float saly = aly.sin();
  float caly = aly.cos();
  float salz = alz.sin();
  float calz = alz.cos();

  float srx = -sbcx * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly) - cbcx * cbcz * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) - cbcx * sbcz * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz);
  acx = -asin(srx);

  float srycrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz) + cbcx * sbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
  float crycrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) + cbcx * cbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
  acy = atan2(srycrx / acx.cos(), crycrx / acx.cos());

  float srzcrx = sbcx * (cblx * cbly * (calz * saly - caly * salx * salz) - cblx * sbly * (caly * calz + salx * saly * salz) + calx * salz * sblx) - cbcx * cbcz * ((caly * calz + salx * saly * salz) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cblz * salz) + cbcx * sbcz * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * cblx * salz * sblz);
  float crzcrx = sbcx * (cblx * sbly * (caly * salz - calz * salx * saly) - cblx * cbly * (saly * salz + caly * calz * salx) + calx * calz * sblx) + cbcx * cbcz * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * calz * cblx * cblz) - cbcx * sbcz * ((saly * salz + caly * calz * salx) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (cbly * cblz + sblx * sbly * sblz) - calx * calz * cblx * sblz);
  acz = atan2(srzcrx / acx.cos(), crzcrx / acx.cos());
}

void BasicLaserOdometry::accumulateRotation(Angle cx, Angle cy, Angle cz,
                                            Angle lx, Angle ly, Angle lz,
                                            Angle& ox, Angle& oy, Angle& oz) {
  float srx = lx.cos() * cx.cos() * ly.sin() * cz.sin() - cx.cos() * cz.cos() * lx.sin() - lx.cos() * ly.cos() * cx.sin();
  ox = -asin(srx);

  float srycrx = lx.sin() * (cy.cos() * cz.sin() - cz.cos() * cx.sin() * cy.sin()) + lx.cos() * ly.sin() * (cy.cos() * cz.cos() + cx.sin() * cy.sin() * cz.sin()) + lx.cos() * ly.cos() * cx.cos() * cy.sin();
  float crycrx = lx.cos() * ly.cos() * cx.cos() * cy.cos() - lx.cos() * ly.sin() * (cz.cos() * cy.sin() - cy.cos() * cx.sin() * cz.sin()) - lx.sin() * (cy.sin() * cz.sin() + cy.cos() * cz.cos() * cx.sin());
  oy = atan2(srycrx / ox.cos(), crycrx / ox.cos());

  float srzcrx = cx.sin() * (lz.cos() * ly.sin() - ly.cos() * lx.sin() * lz.sin()) + cx.cos() * cz.sin() * (ly.cos() * lz.cos() + lx.sin() * ly.sin() * lz.sin()) + lx.cos() * cx.cos() * cz.cos() * lz.sin();
  float crzcrx = lx.cos() * lz.cos() * cx.cos() * cz.cos() - cx.cos() * cz.sin() * (ly.cos() * lz.sin() - lz.cos() * lx.sin() * ly.sin()) - cx.sin() * (ly.sin() * lz.sin() + ly.cos() * lz.cos() * lx.sin());
  oz = atan2(srzcrx / ox.cos(), crzcrx / ox.cos());
}

void BasicLaserOdometry::updateIMU(pcl::PointCloud<pcl::PointXYZ> const& imuTrans) {
  assert(4 == imuTrans.size());
  _imuPitchStart = imuTrans.points[0].x;
  _imuYawStart = imuTrans.points[0].y;
  _imuRollStart = imuTrans.points[0].z;

  _imuPitchEnd = imuTrans.points[1].x;
  _imuYawEnd = imuTrans.points[1].y;
  _imuRollEnd = imuTrans.points[1].z;

  _imuShiftFromStart = imuTrans.points[2];
  _imuVeloFromStart = imuTrans.points[3];
}

void BasicLaserOdometry::process() {
  //smk: If system has not intiaized set the pointclouds and KDTrees and wait for next
  if (!_systemInited) {
    _cornerPointsLessSharp.swap(_lastCornerCloud);
    _surfPointsLessFlat.swap(_lastSurfaceCloud);

    _lastCornerKDTree.setInputCloud(_lastCornerCloud);
    _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);

    _transformSum.rot_x += _imuPitchStart;
    _transformSum.rot_z += _imuRollStart;

    //CUSTOMIZATION
    //FACTOR-GRAPH
    //Estimate Initial attitude from IMU w.r.t Gravity in IMU Frame
    double g;
    gtsam::Rot3 imu_attitude;
    if (!_imuBuffer.estimateAttitudeFromImu(_currentPCLTime.toSec(),
                                            _graphMgr._state.imuBias().accelerometer(),
                                            imu_attitude,
                                            g,
                                            &_graphMgr.setInitGyrBias(),
                                            true)) {
      ROS_WARN("LaserOdometry Failed to Initialize Attitude from IMU - RETRYING ...");
      return;
    }

    //Setup IMU-to-LiDAR and LiDAR-to-IMU transforms
    tf::StampedTransform T_LB, T_BL;
    try {
      _tfListener.lookupTransform(_lidarFrame, _imuFrame, ros::Time(0), T_LB);
      _tfListener.lookupTransform(_imuFrame, _lidarFrame, ros::Time(0), T_BL);
    } catch (tf::TransformException ex) {
      ROS_WARN("%s", ex.what());
    }
    //T_LB
    Eigen::Vector3d t_LB(T_LB.getOrigin().x(), T_LB.getOrigin().y(), T_LB.getOrigin().z());
    Eigen::Quaterniond q_LB(T_LB.getRotation().w(), T_LB.getRotation().x(), T_LB.getRotation().y(), T_LB.getRotation().z());
    q_LB.normalize();
    _T_LB.block<3, 1>(0, 3) = t_LB;
    _T_LB.block<3, 3>(0, 0) = q_LB.toRotationMatrix();
    //T_BL
    invertHomogenousMatrix(_T_LB, _T_BL);
    std::cout << "\033[35mLASER\033[0m IMU-to-LiDAR Extrinsic Calibration:\n"
              << _T_LB << std::endl;
    std::cout << "\033[35mLASER\033[0m LiDAR-to-IMU Extrinsic Calibration:\n"
              << _T_BL << std::endl;

    //Initialize IMU Preintegrator and Factor Graph
    _graphMgr.initImuIntegrator(g);

    //Initialize Factor Graph with prior factor
    gtsam::Pose3 init_pose = gtsam::Pose3::identity();
    if (_initGraphRollPicthFromIMU)
      init_pose = gtsam::Pose3(gtsam::Rot3::Ypr(0.0, imu_attitude.pitch(), imu_attitude.roll()),  //Set yaw to zero
                               gtsam::Point3());
    _graphMgr.initPoseVelocityBiasGraph(_currentPCLTime.toSec(), init_pose);

    //Get LiDAR pose in the initial LiDAR frame - FG gives pose in IMU frame
    gtsam::Pose3 initLiDARPose(_T_LB * _graphMgr._state.navState().pose().matrix() * _T_BL);

    //Initialize pose in LOAM frame from LiDAR pose
    _transformSum.rot_z = Angle(initLiDARPose.rotation().roll());
    _transformSum.rot_x = Angle(initLiDARPose.rotation().pitch());
    std::cout << "\033[35mLASER\033[0m - INIT t(x,y,z): " << initLiDARPose.translation().transpose() << ", RPY(deg): " << initLiDARPose.rotation().rpy().transpose() * (180.0 / M_PI) << "\n";

    //Set TF for debug purposes
    _gravityAttitude.getBasis().setRPY(initLiDARPose.rotation().roll(), initLiDARPose.rotation().pitch(), initLiDARPose.rotation().yaw());
    _gravityAttitudeInit = true;
    //CUSTOMIZATION

    _systemInited = true;
    return;
  }

  pcl::PointXYZI coeff;
  Eigen::Matrix<float, 6, 6> matP;

  _frameCount++;

  //ORIGINAL - Only modified if external prior is available
  _transform.pos -= _imuVeloFromStart * _scanPeriod;

  //DEBUG - Negation for human understanding - reverses direction of pcl alignment
  if (_verboseLevel > 1)
    std::cout << "\033[35mLASER\033[0m Motion Model DELTA (x,y,z): [" << -_transform.pos.z() << " " << -_transform.pos.x() << " " << -_transform.pos.y()
              << "], (r,p,y): [" << _transform.rot_z.deg() << " " << _transform.rot_x.deg() << " " << _transform.rot_y.deg() << "]\n";
  //DEBUG

  //CUTOMIZATION
  //External Prior Integration
  bool priorSuceess = false;
  _isExternalPriorHealthy = false;
  Eigen::Vector3d translation_prior, rotation_prior;
  //Check if Primary Prior is available
  bool primaryPrior = false;
  if (_extPriorAvailable) {
    priorSuceess = calcExternalPrior(translation_prior, rotation_prior);
    //Check health
    if (priorSuceess && (translation_prior.norm() < _odomRelativeTranslationMax))
      _isExternalPriorHealthy = primaryPrior = true;
  }

  //Check if Fallback Prior is Available and Needed
  if (_fallbackExtPriorAvailable && !_isExternalPriorHealthy) {
    priorSuceess = calcExternalPrior(translation_prior, rotation_prior, true);
    //Check health
    if (priorSuceess && (translation_prior.norm() < _odomRelativeTranslationMax))
      _isExternalPriorHealthy = true;
  }

  if ((_extPriorAvailable || _fallbackExtPriorAvailable) && !_isExternalPriorHealthy)
    ROS_WARN_THROTTLE(1.0, "*** LaserOdometry - External Priors failed(x,y,z): [%f %f %f] - Motion Norm(>%f):%f ***",
                      -translation_prior[2], -translation_prior[0], -translation_prior[1], _odomRelativeTranslationMax, translation_prior.norm());

  //Use HEALTHY Prior
  if (_isExternalPriorHealthy) {
    //Translation
    _transform.pos.x() = translation_prior[0];
    _transform.pos.y() = translation_prior[1];
    _transform.pos.z() = translation_prior[2];
    //Rotation - Extra Check
    if (rotation_prior.norm() < _odomRelativeRotationMax) {
      _transform.rot_x = Angle(rotation_prior[0]);
      _transform.rot_y = Angle(rotation_prior[1]);
      _transform.rot_z = Angle(rotation_prior[2]);
    }

    //DEBUG - Negation for human understanding - reverses direction of pcl alignment
    if (_verboseLevel > 1)
      std::cout << "\033[35mLASER\033[0m External Prior DELTA (x,y,z): [" << -translation_prior[2] << " " << -translation_prior[0] << " " << -translation_prior[1]
                << "], (r,p,y): [" << rad2deg(rotation_prior[2]) << " " << rad2deg(rotation_prior[0]) << " " << rad2deg(rotation_prior[1])
                << "], Type: " << (primaryPrior ? "P" : "F") << std::endl;
    //DEBUG
  }
  //CUTOMIZATION

  size_t lastCornerCloudSize = _lastCornerCloud->points.size();
  size_t lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

  if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100) {
    std::vector<int> pointSearchInd(1);
    std::vector<float> pointSearchSqDis(1);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(*_cornerPointsSharp, *_cornerPointsSharp, indices);  //smk: indices contain index values of original points in the pointcloud before renoval, here might be useless as we resized the original cloud
    size_t cornerPointsSharpNum = _cornerPointsSharp->points.size();
    size_t surfPointsFlatNum = _surfPointsFlat->points.size();

    _pointSearchCornerInd1.resize(cornerPointsSharpNum);
    _pointSearchCornerInd2.resize(cornerPointsSharpNum);
    _pointSearchSurfInd1.resize(surfPointsFlatNum);
    _pointSearchSurfInd2.resize(surfPointsFlatNum);
    _pointSearchSurfInd3.resize(surfPointsFlatNum);

    double deltaT = std::numeric_limits<double>::quiet_NaN();
    double deltaR = std::numeric_limits<double>::quiet_NaN();
    bool isDegenerate = false;
    bool error_converged = false;
    size_t iterCount = 0;
    std::clock_t time_start = std::clock();
    for (; iterCount < _maxIterations; ++iterCount) {
      pcl::PointXYZI pointSel, pointProj, tripod1, tripod2, tripod3;
      _laserCloudOri->clear();
      _coeffSel->clear();

      for (int i = 0; i < cornerPointsSharpNum; i++) {
        transformToStart(_cornerPointsSharp->points[i], pointSel);

        if (iterCount % 5 == 0)  //smk: every 5th iteration we try to find the closest point again
        {
          pcl::removeNaNFromPointCloud(*_lastCornerCloud, *_lastCornerCloud, indices);
          _lastCornerKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

          int closestPointInd = -1, minPointInd2 = -1;
          if (pointSearchSqDis[0] < 25) {
            closestPointInd = pointSearchInd[0];
            int closestPointScan = int(_lastCornerCloud->points[closestPointInd].intensity);

            float pointSqDis, minPointSqDis2 = 25;
            for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
              if (int(_lastCornerCloud->points[j].intensity) > closestPointScan + 2.5)  //smk: outside the boundry of scan line ID and relative time in the scan, ideally it should be between 16.00 to 16.1 for scan line 16
              {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

              if (int(_lastCornerCloud->points[j].intensity) > closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }
            for (int j = closestPointInd - 1; j >= 0; j--) {
              if (int(_lastCornerCloud->points[j].intensity) < closestPointScan - 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

              if (int(_lastCornerCloud->points[j].intensity) < closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }
          }

          _pointSearchCornerInd1[i] = closestPointInd;  //smk: from KDTree search
          _pointSearchCornerInd2[i] = minPointInd2;     //smk: from scan line search?
        }

        if (_pointSearchCornerInd2[i] >= 0) {
          tripod1 = _lastCornerCloud->points[_pointSearchCornerInd1[i]];
          tripod2 = _lastCornerCloud->points[_pointSearchCornerInd2[i]];

          float x0 = pointSel.x;
          float y0 = pointSel.y;
          float z0 = pointSel.z;
          float x1 = tripod1.x;
          float y1 = tripod1.y;
          float z1 = tripod1.z;
          float x2 = tripod2.x;
          float y2 = tripod2.y;
          float z2 = tripod2.z;

          float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

          float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

          float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

          float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

          float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

          float ld2 = a012 / l12;  // Eq. (2)

          // TODO: Why writing to a variable that's never read?
          pointProj = pointSel;
          pointProj.x -= la * ld2;
          pointProj.y -= lb * ld2;
          pointProj.z -= lc * ld2;

          float s = 1;
          if (iterCount >= 5) {
            s = 1 - 1.8f * fabs(ld2);
          }

          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

          if (s > 0.1 && ld2 != 0) {
            _laserCloudOri->push_back(_cornerPointsSharp->points[i]);
            _coeffSel->push_back(coeff);
          }
        }
      }

      for (int i = 0; i < surfPointsFlatNum; i++) {
        transformToStart(_surfPointsFlat->points[i], pointSel);

        if (iterCount % 5 == 0) {
          _lastSurfaceKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
          int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
          if (pointSearchSqDis[0] < 25) {
            closestPointInd = pointSearchInd[0];
            int closestPointScan = int(_lastSurfaceCloud->points[closestPointInd].intensity);

            float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
            for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
              if (int(_lastSurfaceCloud->points[j].intensity) > closestPointScan + 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) <= closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              } else {
                if (pointSqDis < minPointSqDis3) {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
            for (int j = closestPointInd - 1; j >= 0; j--) {
              if (int(_lastSurfaceCloud->points[j].intensity) < closestPointScan - 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) >= closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              } else {
                if (pointSqDis < minPointSqDis3) {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
          }

          _pointSearchSurfInd1[i] = closestPointInd;
          _pointSearchSurfInd2[i] = minPointInd2;
          _pointSearchSurfInd3[i] = minPointInd3;
        }

        if (_pointSearchSurfInd2[i] >= 0 && _pointSearchSurfInd3[i] >= 0) {
          tripod1 = _lastSurfaceCloud->points[_pointSearchSurfInd1[i]];
          tripod2 = _lastSurfaceCloud->points[_pointSearchSurfInd2[i]];
          tripod3 = _lastSurfaceCloud->points[_pointSearchSurfInd3[i]];

          float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
          float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
          float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
          float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

          float ps = sqrt(pa * pa + pb * pb + pc * pc);
          pa /= ps;
          pb /= ps;
          pc /= ps;
          pd /= ps;

          float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;  //Eq. (3)??

          // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
          pointProj = pointSel;
          pointProj.x -= pa * pd2;
          pointProj.y -= pb * pd2;
          pointProj.z -= pc * pd2;

          float s = 1;
          if (iterCount >= 5) {
            s = 1 - 1.8f * fabs(pd2) / sqrt(calcPointDistance(pointSel));
          }

          coeff.x = s * pa;
          coeff.y = s * pb;
          coeff.z = s * pc;
          coeff.intensity = s * pd2;

          if (s > 0.1 && pd2 != 0) {
            _laserCloudOri->push_back(_surfPointsFlat->points[i]);
            _coeffSel->push_back(coeff);
          }
        }
      }

      int pointSelNum = _laserCloudOri->points.size();
      if (pointSelNum < 10) {
        continue;
      }

      // smk: matA is the Jacobian, matX is the deltaX() and matB contains residuals of the linearized problem
      Eigen::Matrix<float, Eigen::Dynamic, 6> matA(pointSelNum, 6);
      Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, pointSelNum);
      Eigen::Matrix<float, 6, 6> matAtA;
      Eigen::VectorXf matB(pointSelNum);
      Eigen::Matrix<float, 6, 1> matAtB;
      Eigen::Matrix<float, 6, 1> matX;

      for (int i = 0; i < pointSelNum; i++) {
        const pcl::PointXYZI& pointOri = _laserCloudOri->points[i];
        coeff = _coeffSel->points[i];

        float s = 1;

        float srx = sin(s * _transform.rot_x.rad());
        float crx = cos(s * _transform.rot_x.rad());
        float sry = sin(s * _transform.rot_y.rad());
        float cry = cos(s * _transform.rot_y.rad());
        float srz = sin(s * _transform.rot_z.rad());
        float crz = cos(s * _transform.rot_z.rad());
        float tx = s * _transform.pos.x();
        float ty = s * _transform.pos.y();
        float tz = s * _transform.pos.z();

        float arx = (-s * crx * sry * srz * pointOri.x + s * crx * crz * sry * pointOri.y + s * srx * sry * pointOri.z + s * tx * crx * sry * srz - s * ty * crx * crz * sry - s * tz * srx * sry) * coeff.x + (s * srx * srz * pointOri.x - s * crz * srx * pointOri.y + s * crx * pointOri.z + s * ty * crz * srx - s * tz * crx - s * tx * srx * srz) * coeff.y + (s * crx * cry * srz * pointOri.x - s * crx * cry * crz * pointOri.y - s * cry * srx * pointOri.z + s * tz * cry * srx + s * ty * crx * cry * crz - s * tx * crx * cry * srz) * coeff.z;

        float ary = ((-s * crz * sry - s * cry * srx * srz) * pointOri.x + (s * cry * crz * srx - s * sry * srz) * pointOri.y - s * crx * cry * pointOri.z + tx * (s * crz * sry + s * cry * srx * srz) + ty * (s * sry * srz - s * cry * crz * srx) + s * tz * crx * cry) * coeff.x + ((s * cry * crz - s * srx * sry * srz) * pointOri.x + (s * cry * srz + s * crz * srx * sry) * pointOri.y - s * crx * sry * pointOri.z + s * tz * crx * sry - ty * (s * cry * srz + s * crz * srx * sry) - tx * (s * cry * crz - s * srx * sry * srz)) * coeff.z;

        float arz = ((-s * cry * srz - s * crz * srx * sry) * pointOri.x + (s * cry * crz - s * srx * sry * srz) * pointOri.y + tx * (s * cry * srz + s * crz * srx * sry) - ty * (s * cry * crz - s * srx * sry * srz)) * coeff.x + (-s * crx * crz * pointOri.x - s * crx * srz * pointOri.y + s * ty * crx * srz + s * tx * crx * crz) * coeff.y + ((s * cry * crz * srx - s * sry * srz) * pointOri.x + (s * crz * sry + s * cry * srx * srz) * pointOri.y + tx * (s * sry * srz - s * cry * crz * srx) - ty * (s * crz * sry + s * cry * srx * srz)) * coeff.z;

        float atx = -s * (cry * crz - srx * sry * srz) * coeff.x + s * crx * srz * coeff.y - s * (crz * sry + cry * srx * srz) * coeff.z;

        float aty = -s * (cry * srz + crz * srx * sry) * coeff.x - s * crx * crz * coeff.y - s * (sry * srz - cry * crz * srx) * coeff.z;

        float atz = s * crx * sry * coeff.x - s * srx * coeff.y - s * crx * cry * coeff.z;

        float d2 = coeff.intensity;

        matA(i, 0) = arx;
        matA(i, 1) = ary;
        matA(i, 2) = arz;
        matA(i, 3) = atx;
        matA(i, 4) = aty;
        matA(i, 5) = atz;
        matB(i, 0) = -0.05 * d2;
      }
      matAt = matA.transpose();  //smk: dimension of A is nx6, dimension of B is nx1
      matAtA = matAt * matA;     //smk: for Ax=B, this makes l.h.s A = A'A 6x6
      matAtB = matAt * matB;     //smk: for Ax=B, this makes r.h.s B = A'B 6x1

      matX = matAtA.colPivHouseholderQr().solve(matAtB);  //ORIGINAL smk: now matX is 6x1

      if (iterCount == 0) {
        Eigen::Matrix<float, 1, 6> matE;
        Eigen::Matrix<float, 6, 6> matV;
        Eigen::Matrix<float, 6, 6> matV2;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> esolver(matAtA);
        matE = esolver.eigenvalues().real();
        matV = esolver.eigenvectors().real();

        matV2 = matV;

        isDegenerate = false;
        float eignThre[6] = {_odomDegenEigVal, _odomDegenEigVal, _odomDegenEigVal, _odomDegenEigVal, _odomDegenEigVal, _odomDegenEigVal};

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
        // //Logging
        // _odometryLogFile << (_currentPCLTime.toSec() + _currentPCLTime.toNSec() * 1e-9) << ","
        //                  << matE(0, 0) << "," << matE(0, 1) << "," << matE(0, 2) << ","
        //                  << matE(0, 3) << "," << matE(0, 4) << "," << matE(0, 5) << "\n";
        //Enable VIO passthrough if solution was degenerate
        if (_isExternalPriorHealthy) {
          if (isDegenerate) {
            ROS_WARN_THROTTLE(0.5, "*** LOAM - ODOMETRY Degenerate - Using External Estimate ***");
            _wasDegenerate = true;
            _countDegenerate = 5;
            break;
          } else {
            //Decrement degeneracy counter and reset status
            --_countDegenerate;
            if (_countDegenerate < 1)
              _wasDegenerate = false;

            //Enable/Disable VIO passthrough if previously degenerate
            if (_wasDegenerate) {
              // std::cout << "\033[35m*** VIO passthrough LASER ODOMETRY Degenerate ***\033[0m" << " _countDegenerate: " << _countDegenerate << << std::endl; //DEBUG
              break;
            }
          }
        }
        //CUSTOMIZATION
      }

      if (isDegenerate) {
        Eigen::Matrix<float, 6, 1> matX2(matX);
        matX = matP * matX2;
        if (iterCount == 0)
          ROS_WARN_THROTTLE(0.5, "*** LOAM - ODOMETRY Degenerate ***");
        _wasDegenerate = true;  //CUSTOMIZATION //Added for status monitoring in-case external odometry is not available
      } else
        _wasDegenerate = false;  //CUSTOMIZATION

      _transform.rot_x = _transform.rot_x.rad() + matX(0, 0);
      _transform.rot_y = _transform.rot_y.rad() + matX(1, 0);
      _transform.rot_z = _transform.rot_z.rad() + matX(2, 0);
      _transform.pos.x() += matX(3, 0);
      _transform.pos.y() += matX(4, 0);
      _transform.pos.z() += matX(5, 0);

      if (!std::isfinite(_transform.rot_x.rad()))
        _transform.rot_x = Angle();
      if (!std::isfinite(_transform.rot_y.rad()))
        _transform.rot_y = Angle();
      if (!std::isfinite(_transform.rot_z.rad()))
        _transform.rot_z = Angle();

      if (!std::isfinite(_transform.pos.x()))
        _transform.pos.x() = 0.0;
      if (!std::isfinite(_transform.pos.y()))
        _transform.pos.y() = 0.0;
      if (!std::isfinite(_transform.pos.z()))
        _transform.pos.z() = 0.0;

      deltaR = std::sqrt(std::pow(rad2deg(matX(0, 0)), 2) +
                         std::pow(rad2deg(matX(1, 0)), 2) +
                         std::pow(rad2deg(matX(2, 0)), 2));
      deltaT = std::sqrt(std::pow(matX(3, 0) * 100, 2) +
                         std::pow(matX(4, 0) * 100, 2) +
                         std::pow(matX(5, 0) * 100, 2));
      if (deltaR < _deltaRAbort && deltaT < _deltaTAbort && iterCount > _minIterations) {
        // //CUSTOMIZATION
        // //Calculation of the vairance of the solution
        // Eigen::VectorXf residual = (matA * matX) - matB;
        // float mse = static_cast<float>(residual.transpose() * residual) / (pointSelNum - 6); //smk: matlab nlinfit
        // //float variance = residual.norm() / (pointSelNum - 6);
        // Eigen::Matrix<float, 6, 6> covMat = mse * matAtA.inverse();
        // std::cout << "Odometry, MSE: " << mse << ", covMat diagonal: " << covMat.diagonal().transpose() << std::endl;
        // //CUSTOMIZATION
        error_converged = true;
        break;
      }
    }

    //DEBUG - Optimization Status
    if (_verboseLevel > 0) {
      auto t = static_cast<double>(std::clock() - time_start) / CLOCKS_PER_SEC * 1000.0;
      std::cout << "\033[35mLASER\033[0m - Converged:" << (error_converged ? " \033[32mTrue\033[0m" : "\033[31mFalse\033[0m")
                << ", Degenerate:" << (isDegenerate ? " \033[31mTrue\033[0m" : "\033[32mFalse\033[0m")
                << ", Iter:" << iterCount + 1
                << ", Residual(T/R):" << deltaT << "/" << deltaR
                << ", Time(ms):" << t
                << (t < 75 ? " \033[32mGOOD\033[0m" : " \033[31mSLOW\033[0m") << "\n";
    }
    //DEBUG - Optimization Status

    //CUSTOMIZATION - Update Optimization Status Message
    _optStatusMsg.converged = error_converged;
    _optStatusMsg.translation_residual = deltaT;
    _optStatusMsg.rotation_residual = deltaR;
    _optStatusMsg.degenerate = isDegenerate;
    _optStatusMsg.iteration_count = iterCount + 1;
    _optStatusMsg.iteration_max = _maxIterations;
    _optStatusMsg.optimization_time = static_cast<double>(std::clock() - time_start) / CLOCKS_PER_SEC;
    //Status
    static const double optTimeLimit = 0.1;  //10Hz
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
  }

  /* -----Factor Graph----- */
  //Update Factor graph with IMU and LiDAR pose factors
  updateFactorGraph();

  //Get new pose
  gtsam::Pose3 lidarPose(_T_LB * _graphMgr._state.navState().pose().matrix() * _T_BL); // L_B * Binit_Bnow * B_L = LinitLnow
  //Update IMU bias message for publishing
  tf::vectorEigenToMsg(_graphMgr._state.imuBias().accelerometer(), _imuBiasMsg.linear_acceleration);  //Accelerometer
  tf::vectorEigenToMsg(_graphMgr._state.imuBias().gyroscope(), _imuBiasMsg.angular_velocity);         //Gyroscope
  /* -----Factor Graph----- */

  //Update odometry
  Angle rx, ry, rz;
  accumulateRotation(_transformSum.rot_x,
                     _transformSum.rot_y,
                     _transformSum.rot_z,
                     -_transform.rot_x,
                     -_transform.rot_y.rad() * 1.05,
                     -_transform.rot_z,
                     rx, ry, rz);

  Vector3 v(_transform.pos.x() - _imuShiftFromStart.x(),
            _transform.pos.y() - _imuShiftFromStart.y(),
            _transform.pos.z() * 1.05 - _imuShiftFromStart.z());

  rotateZXY(v, rz, rx, ry);  //smk: _transform.pos vector seems to be rotated and then negated??? (negation might be due to deltaX same as equation 20 in JFR 2018 paper)
  Vector3 trans = _transformSum.pos - v;

  pluginIMURotation(rx, ry, rz,
                    _imuPitchStart, _imuYawStart, _imuRollStart,
                    _imuPitchEnd, _imuYawEnd, _imuRollEnd,
                    rx, ry, rz);

  _transformSum.rot_x = rx;
  _transformSum.rot_y = ry;
  _transformSum.rot_z = rz;
  _transformSum.pos = trans;

  //Factor-graph odometry - LOAM frame
  _transformSum.pos.z() = lidarPose.translation().x();
  _transformSum.pos.x() = lidarPose.translation().y();
  _transformSum.pos.y() = lidarPose.translation().z();
  _transformSum.rot_z = Angle(lidarPose.rotation().roll()); 
  _transformSum.rot_x = Angle(lidarPose.rotation().pitch());
  _transformSum.rot_y = Angle(lidarPose.rotation().yaw());  
  //Factor-graph odometry

  //DEBUG - Negation for human understanding - reverses direction of pcl alignment
  if (_verboseLevel > 1) {
    std::cout << "\033[35mLASER\033[0m Opt-Result DETLA (x,y,z): [" << -_transform.pos.z() << " " << -_transform.pos.x() << " " << -_transform.pos.y()
              << "], (r,p,y): [" << _transform.rot_z.deg() << " " << _transform.rot_x.deg() << " " << _transform.rot_y.deg() << "]\n";
    std::cout << "\033[35mLASER\033[0m Odometry (x,y,z): [" << _transformSum.pos.z() << " " << _transformSum.pos.x() << " " << _transformSum.pos.y()
              << "], (r,p,y): [" << _transformSum.rot_z.deg() << " " << _transformSum.rot_x.deg() << " " << _transformSum.rot_y.deg() << "]\n";
  }
  //DEBUG

  transformToEnd(_cornerPointsLessSharp);
  transformToEnd(_surfPointsLessFlat);

  _cornerPointsLessSharp.swap(_lastCornerCloud);  //smk: _lastCornerCloud = _cornerPointsLessSharp
  _surfPointsLessFlat.swap(_lastSurfaceCloud);

  lastCornerCloudSize = _lastCornerCloud->points.size();
  lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

  if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100) {
    _lastCornerKDTree.setInputCloud(_lastCornerCloud);
    _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);
  }
}

//CUSTOMIZATION
void BasicLaserOdometry::updatePCLTime(const ros::Time& currentTimeStamp) {
  _lastPCLTime = _currentPCLTime;
  _currentPCLTime = currentTimeStamp;
  // //DEBUG
  // std::cout << "\033[35mLASER\033[0m _lastPCLTime: " << _lastPCLTime.sec << "," << _lastPCLTime.nsec
  //           << ", _currentPCLTime: " << _currentPCLTime.sec << "," << _currentPCLTime.nsec
  //           << ", diff: " << (_currentPCLTime - _lastPCLTime).toSec() << std::endl;
  // //DEBUG
}

bool BasicLaserOdometry::calcExternalPrior(Eigen::Vector3d& transPriorXYZ, Eigen::Vector3d& rotPriorXYZ, bool useFallback /*= false*/) {
  transPriorXYZ.setZero();
  rotPriorXYZ.setZero();
  tf::StampedTransform T_S2S1;  //Relative Transform of S1 w.r.t S2 using fixed frame
  tf::StampedTransform T_LS;    //Transform from external Sensor to LiDAR Frame - Extrinsic Calibration
  static ros::Duration time_offset(_extOdomTimeOffset);
  static ros::Duration fallback_time_offset(_fallbackExtOdomTimeOffset);

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
  prior.getBasis().getRPY(e1[2], e1[1], e1[0]);  //XYZ
  prior.getBasis().getRPY(e2[2], e2[1], e2[0], 2);
  //Set smaller rotation solution
  if (e1.norm() < e2.norm())
    rotPriorXYZ << e1[1], e1[0], e1[2];  //YZX
  else
    rotPriorXYZ << e2[1], e2[0], e2[2];

  return true;
}

bool BasicLaserOdometry::updateFactorGraph() {
  //Update keys and timestamps
  auto old_key = _graphMgr._state.key();
  auto new_key = _graphMgr.newStateKey();
  auto old_ts = _lastPCLTime.toSec() + _imuTimeOffset;
  auto new_ts = _currentPCLTime.toSec() + _imuTimeOffset;
  // std::cout << std::fixed << "FACTOR-GRAPH keys: (" << old_key << "/" << new_key << "), ts: (" << old_ts << "/" << new_ts << "), diff:" << new_ts - old_ts << std::endl;

  //Get IMU measurements between pointcloud timestamps and add IMU factor
  IMUMap imu_meas;
  bool success = _imuBuffer.getInterpolatedImuMeasurements(old_ts, new_ts, imu_meas);
  if (success)
    _graphMgr.addImuFactor(old_key, new_key, imu_meas);
  else
    ROS_WARN("LaserOdometry - IMU factor not added to graph");

  //CREATE LiDAR POSE BETWEEN FACTOR
  //Pose delta in LiDAR Frame - LOAM calcualtes transformation to be applied on New pointcloud to align with to Old
  //For odometry we require Old->New transformation. Hence, delta rotation and translation is negated to reverse direction.
  Eigen::Matrix4d delta_lidar = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond dq = Eigen::AngleAxisd(-_transform.rot_y.rad(), Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(-_transform.rot_x.rad(), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(-_transform.rot_z.rad(), Eigen::Vector3d::UnitX());
  dq.normalize();
  delta_lidar.block<3, 3>(0, 0) = dq.toRotationMatrix();                                                           //R
  delta_lidar.block<3, 1>(0, 3) = Eigen::Vector3d(-_transform.pos.z(), -_transform.pos.x(), -_transform.pos.y());  //t

  //Convert to IMU frame
  gtsam::Pose3 delta_imu(_T_BL * delta_lidar * _T_LB);
  
  //Check if Zero Motion and add Zero motion Factor
  bool zeroMotion = false;
  if (_zeroMotionDetection)
    zeroMotion = _graphMgr.zeroMotionFactor(old_key, new_key, delta_imu);

  //Add Pose between Factor if non-zero motion
  if (!zeroMotion)
    _graphMgr.addPoseBetweenFactor(old_key, new_key, delta_imu);

  //Add IMU attitute prior if Zero Motion
  if(_gravityRollPitchFactors && zeroMotion){
    double g;
    gtsam::Rot3 imu_attitude;
    _imuBuffer.estimateAttitudeFromImu(old_ts,
                                       _graphMgr._state.imuBias().accelerometer(),
                                       imu_attitude,
                                       g);
    gtsam::Rot3 current_attitude = _graphMgr._state.navState().attitude();
    auto yawAdjIMUattitude = gtsam::Rot3::Ypr(current_attitude.yaw(), imu_attitude.pitch(), imu_attitude.roll());
    _graphMgr.addGravityRollPitchFactor(old_key, yawAdjIMUattitude);
  }

  //Update graph with factors and calculate new state
  _graphMgr.updateGraphAndState(new_ts, new_key);

  return success;
}
//CUSTOMIZATION

}  // end namespace loam
