#ifndef IMU_MANAGER_HPP_
#define IMU_MANAGER_HPP_

//C++
#include <map>
#include <mutex>

//Eigen
#include <Eigen/Dense>

//GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace loam {
typedef std::map<double, gtsam::Vector6, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> IMUMap;
typedef IMUMap::iterator IMUMapItr;

class ImuManager {
 public:
  //Constructor
  ImuManager()
      : _imuRate(400), _imuTimeForInit(1.0) {
    //Reset IMU Buffer
    _IMUBuffer.clear();
  }

  //Destructor
  ~ImuManager() {}

  //Setters
  void setImuRate(double d) { _imuRate = d; }
  void setImuTimeForInit(double d) {_imuTimeForInit =d;}

  // Add IMU measurement to buffer
  void addToIMUBuffer(double ts, double accX, double accY, double accZ, double gyrX, double gyrY, double gyrZ) {
    //Convert to gtsam type
    gtsam::Vector6 imuMeas;
    imuMeas << accX, accY, accZ, gyrX, gyrY, gyrZ;

    //Add to buffer
    std::lock_guard<std::mutex> lock(_IMUBufferMutex);
    _IMUBuffer[ts] = imuMeas;
  }

  //Get IMU measurements with interpolated values at start and end timestamps
  bool getInterpolatedImuMeasurements(const double& ts_start, const double& ts_end, IMUMap& interpolatedIMUMap) {
    //clear
    interpolatedIMUMap.clear();

    //Get nearest IMUMap iterators corresponding to timestamps
    IMUMapItr s_itr, e_itr;
    std::lock_guard<std::mutex> lock(_IMUBufferMutex);
    bool success = getIMUBufferIteratorsInInterval(ts_start, ts_end, s_itr, e_itr);
    if (success) {
      //Copy in between IMU measurements
      interpolatedIMUMap.insert(s_itr, e_itr);  //Note: Value at e_itr is not inserted

      //Interpolate first element
      if (s_itr->first > ts_start) {
        auto prev_s_itr = s_itr;
        --prev_s_itr;
        gtsam::Vector6 ts_start_meas = interpolateIMUMeasurement(prev_s_itr->first,
                                                                 prev_s_itr->second,
                                                                 ts_start,
                                                                 s_itr->first,
                                                                 s_itr->second);

        //Add Interpolated Value to return IMUMap
        interpolatedIMUMap[ts_start] = ts_start_meas;
      }

      //Add last element
      if (e_itr->first > ts_end) {
        //Interpolate IMU message at timestamp
        auto prev_e_itr = e_itr;
        --prev_e_itr;
        gtsam::Vector6 ts_end_meas = interpolateIMUMeasurement(prev_e_itr->first,
                                                               prev_e_itr->second,
                                                               ts_end,
                                                               e_itr->first,
                                                               e_itr->second);

        interpolatedIMUMap[ts_end] = ts_end_meas;
      } else {
        //Add last actual IMU message
        interpolatedIMUMap[e_itr->first] = e_itr->second;  ////std::map insert doesn't insert last iterator so if e_itr->first > ts_end then it means we are at the end of IMU buffer and we need to insert the last IMU message to the return buffer
        //Extrapolate IMU message at timestamp(ts_end>k), e_itr(k), prev_e_itr(k-1)
        auto prev_e_itr = e_itr;
        --prev_e_itr;
        gtsam::Vector6 ts_end_meas = extrapolateIMUMeasurement(prev_e_itr->first,
                                                               prev_e_itr->second,
                                                               e_itr->first,
                                                               e_itr->second,
                                                               ts_end);
        //Add Extrapolated Value to return IMUMap
        interpolatedIMUMap[ts_end] = ts_end_meas;
      }
      return true;
    } else
      return false;
  }

  //Get iterators to IMU messages in a given time interval
  bool getIMUBufferIteratorsInInterval(const double& ts_start, const double& ts_end, IMUMapItr& s_itr, IMUMapItr& e_itr) {
    //Check if timestamps are in correct order
    if (ts_start >= ts_end) {
      std::cout << "\033[33mIMU-Manager\033[0m IMU Lookup Timestamps are not correct ts_start(" << std::fixed << ts_start << ") >= ts_end(" << ts_end << ")\n";
      return false;
    }

    //Get Iterator Belonging to ts_start
    s_itr = _IMUBuffer.lower_bound(ts_start);
    //Get Iterator Belonging to ts_end
    e_itr = _IMUBuffer.lower_bound(ts_end);

    //Check if it is first value in the buffer which means there is no value before to interpolate with
    if (s_itr == _IMUBuffer.begin()) {
      std::cout << "\033[33mIMU-Manager\033[0m Lookup requires first message of IMU buffer, cannot Interpolate back, Lookup Start/End: "
                << std::fixed << ts_start << "/" << ts_end << ", Buffer Start/End: "
                << _IMUBuffer.begin()->first << "/" << _IMUBuffer.rbegin()->first << std::endl;
      return false;
    }

    //Check if lookup start time is ahead of buffer start time
    if (s_itr == _IMUBuffer.end()) {
      std::cout << "\033[33mIMU-Manager\033[0m IMU Lookup start time ahead latest IMU message in the buffer, lookup: " << ts_start
                << ", latest IMU: " << _IMUBuffer.rbegin()->first << std::endl;
      return false;
    }

    //Check if last value is valid
    if (e_itr == _IMUBuffer.end()) {
      std::cout << "\033[33mIMU-Manager\033[0m Lookup is past IMU buffer, with lookup Start/End: " << std::fixed << ts_start
                << "/" << ts_end << " and latest IMU: " << _IMUBuffer.rbegin()->first << std::endl;
      e_itr = _IMUBuffer.end();
      --e_itr;
    }

    //Check if two IMU messages are different
    if (s_itr == e_itr) {
      std::cout << "\033[33mIMU-Manager\033[0m Not Enough IMU values between timestamps , with Start/End: " << std::fixed << ts_start << "/" << ts_end
                << ", with diff: " << ts_end - ts_start << std::endl;
      return false;
    }

    //If everything is good
    return true;
  }

  //Interpolate IMU measurement between timestamps
  gtsam::Vector6 interpolateIMUMeasurement(const double& ts1, const gtsam::Vector6& meas1, const double& ts2, const double& ts3, const gtsam::Vector6& meas3) {
    double tsDiffRatio = (ts2 - ts1) / (ts3 - ts1);                //(x2-x1)/(x3-x1)
    gtsam::Vector6 meas2 = (meas3 - meas1) * tsDiffRatio + meas1;  //y2 = (y3-y1) * ((x2-x1)/(x3-x1)) + y1

    // std::cout << "------ Interpolate IMU Measurement -----\n";
    // std::cout << std::fixed << "ts1 Message, ts: " << ts1 << ", ACC/GYR: " << meas1.transpose() << std::endl;
    // std::cout << std::fixed << "EST Message, ts: " << ts2 << ", ACC/GYR: " << meas2.transpose() << std::endl;
    // std::cout << std::fixed << "ts3 Message, ts: " << ts3 << ", ACC/GYR: " << meas3.transpose() << std::endl;

    return meas2;
  }

  //Extrapolate IMU measurement between timestamps
  gtsam::Vector6 extrapolateIMUMeasurement(const double& ts1, const gtsam::Vector6& meas1, const double& ts2, const gtsam::Vector6& meas2, const double& ts3) {
    double tsDiffRatio = (ts3 - ts1) / (ts2 - ts1);                //(x2-x1)/(x3-x1)
    gtsam::Vector6 meas3 = (meas2 - meas1) * tsDiffRatio + meas1;  //y3 = (y2-y1) * ((x2-x1)/(x3-x1)) + y1

    // std::cout << "------ Extrapolate IMU Measurement -----\n";
    // std::cout << std::fixed << "ts1 Message, ts: " << ts1 << ", ACC/GYR: " << meas1.transpose() << std::endl;
    // std::cout << std::fixed << "ts2 Message, ts: " << ts2 << ", ACC/GYR: " << meas2.transpose() << std::endl;
    // std::cout << std::fixed << "EST Message, ts: " << ts3 << ", ACC/GYR: " << meas3.transpose() << std::endl;

    return meas3;
  }

  // Determine initial IMU roll/pitch w.r.t to gravity vector and gyro bias
  bool estimateAttitudeFromImu(const double ts, const Eigen::Vector3d& acc_bias, 
                               gtsam::Rot3& init_attitude, double& gravity_magnitude, Eigen::Vector3d *gyr_bias = nullptr,
                               bool output = false) {
    //Get timestamp of first message for lookup
    if (_IMUBuffer.size() < (_imuRate * _imuTimeForInit)) {
      std::cout << "\033[33mIMU-Manager\033[0m NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE\n";
      return false;
    } else {
      //Get IMU Message iterators in the interval
      IMUMap imu_meas;
      double prev_ts = ts - _imuTimeForInit;
      bool success = getInterpolatedImuMeasurements(prev_ts, ts, imu_meas);
      if (success) {
        //Accumulate Acceleration part of IMU Messages
        Eigen::Vector3d acc_mean(0.0, 0.0, 0.0), gyr_mean(0.0, 0.0, 0.0);
        for (auto itr = imu_meas.begin(); itr != imu_meas.end(); ++itr){
          acc_mean += itr->second.head<3>();
          gyr_mean += itr->second.tail<3>();
        }
        //Average IMU measurements and remove bias
        acc_mean /= imu_meas.size();
        acc_mean -= acc_bias;  //remove bias
        //Calculate gyro bias
        gyr_mean /= imu_meas.size();
        if (gyr_bias)
          *gyr_bias = gyr_mean;
        //Set gravity direction (Z-Up same as ROS convention)
        gravity_magnitude = acc_mean.norm();
        Eigen::Vector3d g_unit_vec(0.0, 0.0, 1.0);
        //Normalize gravity vectors to remove the affect of gravity magnitude from place-to-place
        acc_mean.normalize();
        //Calculate robot initial orientation using gravity vector.
        init_attitude = gtsam::Rot3(Eigen::Quaterniond().setFromTwoVectors(acc_mean, g_unit_vec));
        if (output) {
          // std::cout << "\033[33mIMU-Manager\033[0m Gravity Magnitude: " << gravity_magnitude << std::endl;
          // std::cout << "\033[33mIMU-Manager\033[0m Mean IMU Acceleration Vector(x,y,z): " << acc_mean.transpose()
          //           << " - Gravity Unit Vector(x,y,z): " << g_unit_vec.transpose() << std::endl;
          std::cout << "\033[33mIMU-Manager\033[0m IMU Rate(Hz): " << _imuRate <<", Time Interval(s): "<<_imuTimeForInit<<", num meas used: "<<(_imuRate * _imuTimeForInit) << std::endl;
          std::cout << "\033[33mIMU-Manager\033[0m Roll/Pitch/Yaw(deg): " << init_attitude.rpy().transpose() * (180.0 / M_PI) << std::endl;
          std::cout << "\033[33mIMU-Manager\033[0m Gyro bias(x,y,z): " << gyr_mean.transpose() << std::endl;
        }
      } else
        return false;
    }
    return true;
  }

 private:
  std::mutex _IMUBufferMutex;               //Mutex for reading writing IMU buffer
  IMUMap _IMUBuffer;                        //IMU buffer
  double _imuRate;                          //Rate of IMU input (Hz) - Used to calculate minimum measurements used to calculate gravity, initial Roll/Pitch and gyro bias
  double _imuTimeForInit;                   //Multiplied with _imuRate
};
}  // namespace loam
#endif