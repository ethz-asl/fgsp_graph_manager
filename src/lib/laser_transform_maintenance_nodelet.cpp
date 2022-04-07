#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "nodelet/nodelet.h"

#include "loam/TransformMaintenance.h"

class LaserTransformMaintenanceNodelet : public nodelet::Nodelet {
 public:
  LaserTransformMaintenanceNodelet() {}
  ~LaserTransformMaintenanceNodelet() {}

 private:
  //Function
  virtual void onInit() {
    //Get Node Handles
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    //Create Instance and Initialize
    transMaintenance_ = std::make_unique<loam::TransformMaintenance>();
    if (transMaintenance_->setup(nh_, pnh_))
      NODELET_WARN("--- Laser Transform Maintenance Nodelet Initialized ---");
  }
  //Data
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<loam::TransformMaintenance> transMaintenance_;
};

//Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(LaserTransformMaintenanceNodelet, nodelet::Nodelet);