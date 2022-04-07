#include "loam/BasicScanRegistration.h"


namespace loam {

RegistrationParams::RegistrationParams(const float &scanPeriod_,
                                       const int &imuHistorySize_,
                                       const int &nFeatureRegions_,
                                       const int &curvatureRegion_,
                                       const int &maxCornerSharp_,
                                       const int &maxSurfaceFlat_,
                                       const float &lessFlatFilterSize_,
                                       const float &surfaceCurvatureThreshold_)
    : scanPeriod(scanPeriod_),
      imuHistorySize(imuHistorySize_),
      nFeatureRegions(nFeatureRegions_),
      curvatureRegion(curvatureRegion_),
      maxCornerSharp(maxCornerSharp_),
      maxCornerLessSharp(10 * maxCornerSharp_),
      maxSurfaceFlat(maxSurfaceFlat_),
      lessFlatFilterSize(lessFlatFilterSize_),
      surfaceCurvatureThreshold(surfaceCurvatureThreshold_){};

void BasicScanRegistration::processScanlines(const Time &scanTime, std::vector<pcl::PointCloud<pcl::PointXYZI>> const &laserCloudScans) {
  // reset internal buffers and set IMU start state based on current scan time
  reset(scanTime);

  // construct sorted full resolution cloud - smk:all points from the scans are ring-wise inserted into a single pcl and _scanIndices contains the starting and ending point of each scan in the single pcl
  size_t cloudSize = 0;
  for (int i = 0; i < laserCloudScans.size(); i++)  //smk:laserCloudScans.size() = # of scanIDs
  {
    _laserCloud += laserCloudScans[i];
    IndexRange range(cloudSize, 0);
    cloudSize += laserCloudScans[i].size();
    range.second = cloudSize > 0 ? cloudSize - 1 : 0;
    _scanIndices.push_back(range);
  }

  extractFeatures();
  updateIMUTransform();
}

bool BasicScanRegistration::configure(const RegistrationParams &config) {
  //Update Configuration
  _config = config;
  //Set voxelgrid filter
  _downSizeFilter.setLeafSize(_config.lessFlatFilterSize, _config.lessFlatFilterSize, _config.lessFlatFilterSize);
  //Check IMU Buffer /*Deprecated*/
  _imuHistory.ensureCapacity(_config.imuHistorySize);
  return true;
}

void BasicScanRegistration::reset(const Time &scanTime) {
  _scanTime = scanTime;

  // re-initialize IMU start index and state
  _imuIdx = 0;
  if (hasIMUData())
  {
    interpolateIMUStateFor(0, _imuStart);
  }

  // clear internal cloud buffers at the beginning of a sweep
  if (true /*newSweep*/) {
    _sweepStart = scanTime;  //smk: means _sweepStart = scanTime = laserCloudMsg->header.stamp

    // clear cloud buffers
    _laserCloud.clear();
    _cornerPointsSharp.clear();
    _cornerPointsLessSharp.clear();
    _surfacePointsFlat.clear();
    _surfacePointsLessFlat.clear();
    _surfPointsLessFlatScanDS.clear();
    _invalidSharpFeatures.clear();

    // clear scan indices vector
    _scanIndices.clear();
  }
}

void BasicScanRegistration::updateIMUData(Vector3 &acc, IMUState &newState) {
  if (_imuHistory.size() > 0) {
    // accumulate IMU position and velocity over time
    rotateZXY(acc, newState.roll, newState.pitch, newState.yaw);  //smk: LOAM doesn't follow camera convention but Z-Forward, X-Left and Y-Up

    const IMUState &prevState = _imuHistory.last();
    float timeDiff = toSec(newState.stamp - prevState.stamp);
    newState.position = prevState.position + (prevState.velocity * timeDiff) + (0.5 * acc * timeDiff * timeDiff);
    newState.velocity = prevState.velocity + acc * timeDiff;
  }

  _imuHistory.push(newState);
}

void BasicScanRegistration::projectPointToStartOfSweep(pcl::PointXYZI &point, float relTime) {
  // project point to the start of the sweep using corresponding IMU data
  if (hasIMUData()) {
    setIMUTransformFor(relTime);
    transformToStartIMU(point);
  }
}


void BasicScanRegistration::setIMUTransformFor(const float &relTime) {
  interpolateIMUStateFor(relTime, _imuCur);

  float relSweepTime = toSec(_scanTime - _sweepStart) + relTime;
  _imuPositionShift = _imuCur.position - _imuStart.position - _imuStart.velocity * relSweepTime;
}

void BasicScanRegistration::transformToStartIMU(pcl::PointXYZI &point) {
  // rotate point to global IMU system
  rotateZXY(point, _imuCur.roll, _imuCur.pitch, _imuCur.yaw);

  // add global IMU position shift
  point.x += _imuPositionShift.x();
  point.y += _imuPositionShift.y();
  point.z += _imuPositionShift.z();

  // rotate point back to local IMU system relative to the start IMU state
  rotateYXZ(point, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);
}

void BasicScanRegistration::interpolateIMUStateFor(const float &relTime, IMUState &outputState) {
  double timeDiff = toSec(_scanTime - _imuHistory[_imuIdx].stamp) + relTime;  //smk: time difference between capture time of point and the IMU message

  //smk: trying to find index of IMU message that came AFTER the scanned point i.e. increment _imuIdx until timeDiff becomes neagtive
  while (_imuIdx < _imuHistory.size() - 1 && timeDiff > 0) {
    _imuIdx++;
    timeDiff = toSec(_scanTime - _imuHistory[_imuIdx].stamp) + relTime;
  }

  if (_imuIdx == 0 || timeDiff > 0)
  {
    outputState = _imuHistory[_imuIdx];
  } else {
    float ratio = -timeDiff / toSec(_imuHistory[_imuIdx].stamp - _imuHistory[_imuIdx - 1].stamp);  //smk: ratio of time difference of point and current IMU message to time differenec of current and previous IMU message
    IMUState::interpolate(_imuHistory[_imuIdx], _imuHistory[_imuIdx - 1], ratio, outputState);     //smk: if time difference of point and current IMU message is very small ratio will be zero hence we trust this IMU message
    //IMUState::interpolate(_imuHistory[_imuIdx-1], _imuHistory[_imuIdx], ratio, outputState); //smk: this should be like this as it makes more sense?
  }
}

void BasicScanRegistration::extractFeatures(const uint16_t &beginIdx) {
  // extract features from individual scans
  size_t nScans = _scanIndices.size();
  for (size_t i = beginIdx; i < nScans; i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
    size_t scanStartIdx = _scanIndices[i].first;
    size_t scanEndIdx = _scanIndices[i].second;

    // skip empty scans - Check if minimum number of points to calculate curvature are available
    if (scanEndIdx <= scanStartIdx + 2 * _config.curvatureRegion)
      continue;

    // Quick&Dirty fix for relative point time calculation without IMU data
    /*float scanSize = scanEndIdx - scanStartIdx + 1;
    for (int j = scanStartIdx; j <= scanEndIdx; j++) {
      _laserCloud[j].intensity = i + _scanPeriod * (j - scanStartIdx) / scanSize;
    }*/

    // reset scan buffers
    setScanBuffersFor(scanStartIdx, scanEndIdx);

    // extract features from equally sized scan regions
    for (int j = 0; j < _config.nFeatureRegions; j++) {
      size_t sp = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - j) + (scanEndIdx - _config.curvatureRegion) * j) / _config.nFeatureRegions;
      size_t ep = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - 1 - j) + (scanEndIdx - _config.curvatureRegion) * (j + 1)) / _config.nFeatureRegions - 1;

      // skip empty regions
      if (ep <= sp)
      {
        continue;
      }

      size_t regionSize = ep - sp + 1;

      // reset region buffers //smk: this calculates the curvature for points in the scans
      setRegionBuffersFor(sp, ep);

      // extract corner features
      int largestPickedNum = 0;
      for (size_t k = regionSize; k > 0 && largestPickedNum < _config.maxCornerLessSharp;)  //smk: maxCornerLessSharp = 10*maxCornerSharp
      {
        size_t idx = _regionSortIndices[--k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        //smk: pick sharp points, the sharp and less sharp need to be above "surfaceCurvatureThreshold", technically there is no difference in btw the two only some of them are marked are sharp to reduce data for laser odometry
        if (_scanNeighborPicked[scanIdx] == 0 && _regionCurvature[regionIdx] > _config.surfaceCurvatureThreshold) {
          // Check if feature is in valid azimuth & polar ranges
          bool isFeatureInvalid = false;
          if (_config.checkInvalidFeatureRegions)
            isFeatureInvalid = checkFeatureInvalid(_laserCloud[idx]);

          //Add Valid Feature
          if (!isFeatureInvalid) {
            ++largestPickedNum;
            if (largestPickedNum <= _config.maxCornerSharp) {
              _regionLabel[regionIdx] = CORNER_SHARP;
              _cornerPointsSharp.push_back(_laserCloud[idx]);
            } else {
              _regionLabel[regionIdx] = CORNER_LESS_SHARP;
            }
            _cornerPointsLessSharp.push_back(_laserCloud[idx]);

            markAsPicked(idx, scanIdx);
          } else if (_config.publishDebugArrowsToRemovedFeatures)  //Skip point and add invalid feature point for debugging
            _invalidSharpFeatures.push_back(_laserCloud[idx]);
        }
      }

      // extract flat surface features //smk: points that are less than "surfaceCurvatureThreshold" and a certain number of them are marked as FLAT(according to per region 4 features), rest are marked as less flat
      int smallestPickedNum = 0;
      for (int k = 0; k < regionSize && smallestPickedNum < _config.maxSurfaceFlat; k++)
      {
        size_t idx = _regionSortIndices[k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] == 0 && _regionCurvature[regionIdx] < _config.surfaceCurvatureThreshold) {
          smallestPickedNum++;
          _regionLabel[regionIdx] = SURFACE_FLAT;
          _surfacePointsFlat.push_back(_laserCloud[idx]);

          markAsPicked(idx, scanIdx);
        }
      }

      // extract less flat surface features
      for (int k = 0; k < regionSize; k++)
      {
        if (_regionLabel[k] <= SURFACE_LESS_FLAT)
        {
          surfPointsLessFlatScan->push_back(_laserCloud[sp + k]);
        }
      }
    }

    // down size less flat surface point cloud of current scan
    _surfPointsLessFlatScanDS.clear();
    _downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    _downSizeFilter.filter(_surfPointsLessFlatScanDS);
    
    // accumulate downsampled less flat points across scan lines
    _surfacePointsLessFlat += _surfPointsLessFlatScanDS;
  }

  // ////DEBUG
  // std::cout << "ScanRegistration Features, CORNER_SHARP:" << _cornerPointsSharp.size()
  //           << ", CORNER_LESS_SHARP:" << _cornerPointsLessSharp.size()
  //           << ", SURFACE_FLAT:" << _surfacePointsFlat.size()
  //           << ", SURFACE_LESS_FLAT:" << _surfacePointsLessFlat.size()
  //           << ", Total:" << _cornerPointsSharp.size() + _cornerPointsLessSharp.size() + _surfacePointsFlat.size() + _surfacePointsLessFlat.size() << std::endl;
  // //DEBUG
}

void BasicScanRegistration::updateIMUTransform() {
  _imuTrans[0].x = _imuStart.pitch.rad();
  _imuTrans[0].y = _imuStart.yaw.rad();
  _imuTrans[0].z = _imuStart.roll.rad();

  _imuTrans[1].x = _imuCur.pitch.rad();
  _imuTrans[1].y = _imuCur.yaw.rad();
  _imuTrans[1].z = _imuCur.roll.rad();

  Vector3 imuShiftFromStart = _imuPositionShift;
  rotateYXZ(imuShiftFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans[2].x = imuShiftFromStart.x();
  _imuTrans[2].y = imuShiftFromStart.y();
  _imuTrans[2].z = imuShiftFromStart.z();

  Vector3 imuVelocityFromStart = _imuCur.velocity - _imuStart.velocity;
  rotateYXZ(imuVelocityFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans[3].x = imuVelocityFromStart.x();
  _imuTrans[3].y = imuVelocityFromStart.y();
  _imuTrans[3].z = imuVelocityFromStart.z();
}

void BasicScanRegistration::setRegionBuffersFor(const size_t &startIdx, const size_t &endIdx) {
  // resize buffers
  size_t regionSize = endIdx - startIdx + 1;
  _regionCurvature.resize(regionSize);
  _regionSortIndices.resize(regionSize);
  _regionLabel.assign(regionSize, SURFACE_LESS_FLAT);  //smk:every point as a SURFACE_LESS_FLAT initially

  // calculate point curvatures and reset sort indices //smk: looping through # number of neighbor points(curvatureRegion) and calculate squared difference
  float pointWeight = -2 * _config.curvatureRegion;

  for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++)
  {
    float diffX = pointWeight * _laserCloud[i].x;
    float diffY = pointWeight * _laserCloud[i].y;
    float diffZ = pointWeight * _laserCloud[i].z;

    for (int j = 1; j <= _config.curvatureRegion; j++)
    {
      diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
      diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
      diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
    }

    _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    _regionSortIndices[regionIdx] = i;
  }

  // sort point curvatures
  for (size_t i = 1; i < regionSize; i++)
  {
    for (size_t j = i; j >= 1; j--)
    {
      if (_regionCurvature[_regionSortIndices[j] - startIdx] < _regionCurvature[_regionSortIndices[j - 1] - startIdx])
      {
        std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);
      }
    }
  }
}

void BasicScanRegistration::setScanBuffersFor(const size_t &startIdx, const size_t &endIdx) {
  // resize buffers
  size_t scanSize = endIdx - startIdx + 1;
  _scanNeighborPicked.assign(scanSize, 0);

  // mark unreliable points as picked -- smk: points that have depth difference greater than 0.1 meters(?) are set as already picked so they can be ignored later
  for (size_t i = startIdx + _config.curvatureRegion; i < endIdx - _config.curvatureRegion; i++) {
    const pcl::PointXYZI &previousPoint = (_laserCloud[i - 1]);
    const pcl::PointXYZI &point = (_laserCloud[i]);
    const pcl::PointXYZI &nextPoint = (_laserCloud[i + 1]);

    float diffNext = calcSquaredDiff(nextPoint, point);

    if (diffNext > 0.1)
    {
      float depth1 = calcPointDistance(point);
      float depth2 = calcPointDistance(nextPoint);

      if (depth1 > depth2)
      {
        float weighted_distance = std::sqrt(calcSquaredDiff(nextPoint, point, depth2 / depth1)) / depth2;

        if (weighted_distance < 0.1)
        {
          std::fill_n(&_scanNeighborPicked[i - startIdx - _config.curvatureRegion], _config.curvatureRegion + 1, 1);

          continue;
        }
      }
      else
      {
        float weighted_distance = std::sqrt(calcSquaredDiff(point, nextPoint, depth1 / depth2)) / depth1;

        if (weighted_distance < 0.1)
        {
          std::fill_n(&_scanNeighborPicked[i - startIdx + 1], _config.curvatureRegion + 1, 1);
        }
      }
    }

    float diffPrevious = calcSquaredDiff(point, previousPoint);
    float dis = calcSquaredPointDistance(point);

    if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis)  //smk:this will reject a point if the difference in distance or depth of point and its neighbors is outside a bound i.e. rejecting very sharp ramp like points
    {
      _scanNeighborPicked[i - startIdx] = 1;
    }
  }
}

void BasicScanRegistration::markAsPicked(const size_t &cloudIdx, const size_t &scanIdx) {
  _scanNeighborPicked[scanIdx] = 1;

  //smk: not only mark the point as picked but mark all the points used in the curvatureRegion around it as picked as well
  for (int i = 1; i <= _config.curvatureRegion; i++) {
    if (calcSquaredDiff(_laserCloud[cloudIdx + i], _laserCloud[cloudIdx + i - 1]) > 0.05) {
      break;
    }

    _scanNeighborPicked[scanIdx + i] = 1;
  }

  for (int i = 1; i <= _config.curvatureRegion; i++)
  {
    if (calcSquaredDiff(_laserCloud[cloudIdx - i], _laserCloud[cloudIdx - i + 1]) > 0.05)
    {
      break;
    }

    _scanNeighborPicked[scanIdx - i] = 1;
  }
}

// CUSTOMIZATION
bool BasicScanRegistration::checkFeatureInvalid(const pcl::PointXYZI& point) {
  bool featureInvalid = false;
  if (_config.noFeatureAzimuthBoundaries.size() > 0) {
    float azimuth = std::atan2(point.x, point.z);
    for (uint i = 0; i < _config.noFeatureAzimuthBoundaries.size(); i += 2) {
      //Check if feature is in valid Azimuth range
      featureInvalid = checkBounds(azimuth, _config.noFeatureAzimuthBoundaries[i], _config.noFeatureAzimuthBoundaries[i + 1]);
      //Check if feature is in valid Polar range
      // if (featureInvalid && _config.noFeaturePolarBoundaries.size() > i) {
      //   float polar = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
      //   featureInvalid = checkBounds(polar, _config.noFeaturePolarBoundaries[i], _config.noFeaturePolarBoundaries[i + 1]);
      // }
      if (featureInvalid)
        return true;
    }
  }
  return false;
}

bool BasicScanRegistration::checkBounds(const float angle, const float lowerBound, const float upperBound) {
  if (lowerBound < upperBound) {
    if (angle > lowerBound && angle < upperBound)
      return true;
  } else {
    if (angle < lowerBound && angle > upperBound)
      return true;
  }
  return false;
}
// CUSTOMIZATION

}  // namespace loam