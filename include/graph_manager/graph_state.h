#ifndef GRAPH_STATE_HPP_
#define GRAPH_STATE_HPP_

//C++
#include <memory>
#include <mutex>

//GTSAM
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

namespace loam {
// Class defining Robot State
class State {
 private:
  gtsam::Key key_ = 0;                    // key
  double ts_ = 0.0;                       // timestamp
  gtsam::NavState navState_;              // pose, velocity
  gtsam::imuBias::ConstantBias imuBias_;  // imu bias
  mutable std::mutex stateMutex_;

 public:
  State() = default;
  explicit State(const gtsam::Key key, const double ts, const gtsam::NavState& navState)
      : key_(key), ts_(ts), navState_(navState) {}
  State(const State& other)
      : key_(other.key_), ts_(other.ts_), navState_(other.navState_), imuBias_(other.imuBias_) {}
  
  //Overload assignment operator
  State& operator=(const State& other) {
    key_ = other.key_;
    ts_ = other.ts_;
    navState_ = other.navState_;
    imuBias_ = other.imuBias_;
    return *this;
  }

  //Overload < operator
  bool operator<(const State& rhs) const {
    return ts_ < rhs.ts_;
  }

  //Accessors
  const auto key() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return key_;
  }
  const auto ts() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return ts_;
  }
  const auto& navState() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return navState_;
  }
  const auto& imuBias() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return imuBias_;
  }

  //Update state Graph Key and Timestamp
  void updateKeyAndTimestamp(const gtsam::Key key, const double ts) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
  }

  //Update state Graph Key, Timestamp and NavState(Pose+Velocity)
  void updateNavState(const gtsam::Key key, const double ts, const gtsam::NavState& navState) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
    navState_ = navState;
  }

  //Update state Graph Key, Timestamp and IMU Bias estimate
  void updateImuBias(const gtsam::Key key, const double ts, const gtsam::imuBias::ConstantBias& imuBias) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
    imuBias_ = imuBias;
  }

  //Update state Graph Key, Timestamp, NavState(Pose+Velocity) and IMU Bias estimate
  void updateNavStateAndBias(const gtsam::Key key, const double ts, const gtsam::NavState& navState, const gtsam::imuBias::ConstantBias& imuBias) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
    navState_ = navState;
    imuBias_ = imuBias;
  }
};
using StatePtr = std::unique_ptr<State>;

}  // namespace loam
#endif
