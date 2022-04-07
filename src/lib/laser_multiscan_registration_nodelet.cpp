#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "nodelet/nodelet.h"

#include "loam/MultiScanRegistration.h"

class LaserMultiScanRegistrationNodelet : public nodelet::Nodelet {
 public:
  LaserMultiScanRegistrationNodelet() {}
  ~LaserMultiScanRegistrationNodelet() {}

 private:
  //Function
  virtual void onInit() {
    //Get Multi-Thread Node Handles
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    //Create Instance & Initialize
    multiScan_ = std::make_unique<loam::MultiScanRegistration>();
    if (multiScan_->setup(nh_, pnh_))
      NODELET_WARN("--- Laser Multiscan Registration Nodelet Initialized ---");
  }
  //Data
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<loam::MultiScanRegistration> multiScan_;
};

//Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(LaserMultiScanRegistrationNodelet, nodelet::Nodelet);