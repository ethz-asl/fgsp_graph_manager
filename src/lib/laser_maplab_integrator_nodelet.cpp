#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "nodelet/nodelet.h"

#include "loam/MaplabIntegrator.hpp"

class LaserMaplabIntegratorNodelet : public nodelet::Nodelet {
 public:
  LaserMaplabIntegratorNodelet() {}
  ~LaserMaplabIntegratorNodelet() {}

 private:
  //Function
  virtual void onInit() {
    //Get Node Handles
    nh_ = getMTNodeHandle();
    pnh_ = getMTPrivateNodeHandle();

    //Create Instance & Initialize
    maplabIntegrator_ = std::make_unique<loam::MaplabIntegrator>();
    if (maplabIntegrator_->setup(nh_, pnh_))
      NODELET_WARN("--- Laser Maplab Integrator Nodelet Initialized ---");
  }
  //Data
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<loam::MaplabIntegrator> maplabIntegrator_;
};

//Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(LaserMaplabIntegratorNodelet, nodelet::Nodelet);