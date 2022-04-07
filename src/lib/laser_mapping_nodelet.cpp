#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "nodelet/nodelet.h"

#include "loam/LaserMapping.h"

class LaserMappingNodelet : public nodelet::Nodelet {
 public:
  LaserMappingNodelet() {}
  ~LaserMappingNodelet() {}

 private:
  //Function
  virtual void onInit() {
    //Get Node Handles
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    //Create Instance & Initialize
    laserMapping_ = std::make_unique<loam::LaserMapping>(0.1);
    if (laserMapping_->setup(nh_, pnh_))
      NODELET_WARN("--- Laser Mapping Nodelet Initialized ---");
  }
  //Data
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<loam::LaserMapping> laserMapping_;
};

//Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(LaserMappingNodelet, nodelet::Nodelet);