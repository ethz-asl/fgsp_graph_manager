#include <ros/ros.h>

#include "loam/LaserMapping.h"

/** Main node entry point. */
int main(int argc, char** argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  //Create Instance & Initialize
  loam::LaserMapping laserMapping(0.1);
  if (laserMapping.setup(node, privateNode))
    ros::spin();

  return 0;
}
