#include <ros/ros.h>

#include "loam/TransformMaintenance.h"

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "transformMaintenance");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  //Create Instance & Initialize
  loam::TransformMaintenance transMaintenance;
  if (transMaintenance.setup(node, privateNode))
    ros::spin();

  return 0;
}
