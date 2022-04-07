#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "nodelet/nodelet.h"

#include "loam/LaserOdometry.h"

class LaserOdometryNodelet : public nodelet::Nodelet {
 public:
  LaserOdometryNodelet() {}
  ~LaserOdometryNodelet() {}

 private:
  //Function
  virtual void onInit() {
    //Get Node Handles
    nh_ = getMTNodeHandle();
    pnh_ = getMTPrivateNodeHandle();

    //Create Instance & Initialize
    laserOdom_ = std::make_unique<loam::LaserOdometry>(0.1);
    if (laserOdom_->setup(nh_, pnh_))
      NODELET_WARN("--- Laser Odometry Nodelet Initialized ---");
  }
  //Data
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<loam::LaserOdometry> laserOdom_;
};

//Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(LaserOdometryNodelet, nodelet::Nodelet);