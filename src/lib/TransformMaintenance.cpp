// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "loam/TransformMaintenance.h"

#include <tf/transform_listener.h>

namespace loam {

TransformMaintenance::TransformMaintenance() {
  // initialize odometry and odometry tf messages
  _laserOdometry2.header.frame_id = "camera_init";
  _laserOdometry2.child_frame_id = "camera";

  _laserOdometryTrans2.frame_id_ = _laserOdometry2.header.frame_id;
  _laserOdometryTrans2.child_frame_id_ = _laserOdometry2.child_frame_id;
}

bool TransformMaintenance::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  // advertise integrated laser odometry topic
  _pubLaserOdometry2 = node.advertise<nav_msgs::Odometry>("/integrated_to_init", 5);

  //Synced scan-to-scan odometry subscriber
  _subLaserOdom.subscribe(node, "/laser_odom_to_init", 10);
  _subLaserOdomStatus.subscribe(node, "/laser_odom_optimization_status", 10);
  _laserSyncPtr.reset(new message_filters::Synchronizer<_syncPolicy>(_syncPolicy(10), _subLaserOdom, _subLaserOdomStatus));
  _laserSyncPtr->registerCallback(boost::bind(&TransformMaintenance::syncLaserOdomHandler, this, _1, _2));
  //Synced scan-to-map odometry subscriber
  _subMapOdom.subscribe(node, "/aft_mapped_to_init", 10);
  _subMapOdomStatus.subscribe(node, "/laser_mapping_optimization_status", 10);
  _mapSyncPtr.reset(new message_filters::Synchronizer<_syncPolicy>(_syncPolicy(10), _subMapOdom, _subMapOdomStatus));
  _mapSyncPtr->registerCallback(boost::bind(&TransformMaintenance::syncMapOdomHandler, this, _1, _2));
  //Combined odometry status publisher
  _pubOdomStatus = node.advertise<OptStatus>("/laser_integrated_status", 5);

  // ARL Coordinate Frame Facilitators
  _laserOdometry2_CORRECTED.header.frame_id = "camera_init_CORRECTED";
  _laserOdometry2_CORRECTED.child_frame_id = "camera_CORRECTED";

  _gm_laserTransformStamped2_CORRECTED.header.frame_id = _laserOdometry2_CORRECTED.header.frame_id;
  _gm_laserTransformStamped2_CORRECTED.child_frame_id = _laserOdometry2_CORRECTED.child_frame_id;

  _pubLaserTransformStamped2_CORRECTED = node.advertise<geometry_msgs::TransformStamped>("/loam_transform_CORRECTED", 5);
  _pubLaserOdometry2_CORRECTED = node.advertise<nav_msgs::Odometry>("/integrated_to_init_CORRECTED", 5);
  // END ARL Coordinate Frame Correction

  //CUSTOMIZATION
  // Initialize path message and publisher
  _pathMsg.header.frame_id = _laserOdometry2_CORRECTED.header.frame_id;
  _pathMsg.poses.reserve(20000);  //30 minutes path @ 10Hz
  _pubPath = node.advertise<nav_msgs::Path>("/laser_integrated_path", 5);
  //CUSTOMIZATION

  return true;
}

void TransformMaintenance::syncLaserOdomHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry, const OptStatusConstPtr& laserStatus){
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  updateOdometry(-pitch, -yaw, roll,
                 laserOdometry->pose.pose.position.x,
                 laserOdometry->pose.pose.position.y,
                 laserOdometry->pose.pose.position.z);

  transformAssociateToMap();

  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformMapped()[2], -transformMapped()[0], -transformMapped()[1]);

  _laserOdometry2.header.stamp = laserOdometry->header.stamp;
  _laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
  _laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
  _laserOdometry2.pose.pose.orientation.z = geoQuat.x;
  _laserOdometry2.pose.pose.orientation.w = geoQuat.w;
  _laserOdometry2.pose.pose.position.x = transformMapped()[3];
  _laserOdometry2.pose.pose.position.y = transformMapped()[4];
  _laserOdometry2.pose.pose.position.z = transformMapped()[5];
  _pubLaserOdometry2.publish(_laserOdometry2);

  _laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
  _laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  _laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped()[3], transformMapped()[4], transformMapped()[5]));
  _tfBroadcaster2.sendTransform(_laserOdometryTrans2);

  // ARL Coordinate Frame Facilitators
  _gm_laserTransformStamped2_CORRECTED.header.stamp = laserOdometry->header.stamp;
  _gm_laserTransformStamped2_CORRECTED.transform.rotation.x = geoQuat.x;
  _gm_laserTransformStamped2_CORRECTED.transform.rotation.y = -geoQuat.y;
  _gm_laserTransformStamped2_CORRECTED.transform.rotation.z = -geoQuat.z;
  _gm_laserTransformStamped2_CORRECTED.transform.rotation.w = geoQuat.w;
  _gm_laserTransformStamped2_CORRECTED.transform.translation.x = transformMapped()[5];
  _gm_laserTransformStamped2_CORRECTED.transform.translation.y = transformMapped()[3];
  _gm_laserTransformStamped2_CORRECTED.transform.translation.z = transformMapped()[4];

  _pubLaserTransformStamped2_CORRECTED.publish(_gm_laserTransformStamped2_CORRECTED);
  _tfBroadcaster2.sendTransform(_gm_laserTransformStamped2_CORRECTED);

  _laserOdometry2_CORRECTED.header.stamp = laserOdometry->header.stamp;
  _laserOdometry2_CORRECTED.pose.pose.orientation.x = _gm_laserTransformStamped2_CORRECTED.transform.rotation.x;
  _laserOdometry2_CORRECTED.pose.pose.orientation.y = _gm_laserTransformStamped2_CORRECTED.transform.rotation.y;
  _laserOdometry2_CORRECTED.pose.pose.orientation.z = _gm_laserTransformStamped2_CORRECTED.transform.rotation.z;
  _laserOdometry2_CORRECTED.pose.pose.orientation.w = _gm_laserTransformStamped2_CORRECTED.transform.rotation.w;
  _laserOdometry2_CORRECTED.pose.pose.position.x = _gm_laserTransformStamped2_CORRECTED.transform.translation.x;
  _laserOdometry2_CORRECTED.pose.pose.position.y = _gm_laserTransformStamped2_CORRECTED.transform.translation.y;
  _laserOdometry2_CORRECTED.pose.pose.position.z = _gm_laserTransformStamped2_CORRECTED.transform.translation.z;

  _pubLaserOdometry2_CORRECTED.publish(_laserOdometry2_CORRECTED);
  // END ARL Coordinate Frame Correction

  //CUSTOMIZATION
  // Publish odometry status message based on if scan-to-scan or scan-to-map alignment was degenerate
  if (_pubOdomStatus.getNumSubscribers() > 0) {
    OptStatus optMsg;
    optMsg.header.stamp = laserOdometry->header.stamp;
    //Status degeneracy status
    optMsg.degenerate = (laserStatus->degenerate || _isMappingDegenerate);
    //Publish
    _pubOdomStatus.publish(optMsg);
  }

  // Publish Path
  if (_pubPath.getNumSubscribers() > 0) {
    //Pose Message
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = _laserOdometry2_CORRECTED.header.frame_id;
    poseMsg.header.stamp = laserOdometry->header.stamp;;
    poseMsg.pose.position.x = transformMapped()[5];
    poseMsg.pose.position.y = transformMapped()[3];
    poseMsg.pose.position.z = transformMapped()[4];
    poseMsg.pose.orientation.x = geoQuat.x;
    poseMsg.pose.orientation.y = -geoQuat.y;
    poseMsg.pose.orientation.z = -geoQuat.z;
    poseMsg.pose.orientation.w = geoQuat.w;
    // Path message
    _pathMsg.header.stamp = laserOdometry->header.stamp;
    _pathMsg.poses.push_back(poseMsg);
    //Publish
    _pubPath.publish(_pathMsg);
  }
  //CUSTOMIZATION


}

void TransformMaintenance::syncMapOdomHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped, const OptStatusConstPtr& mapStatus) {
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  updateMappingTransform(-pitch, -yaw, roll,
                         odomAftMapped->pose.pose.position.x,
                         odomAftMapped->pose.pose.position.y,
                         odomAftMapped->pose.pose.position.z,

                         odomAftMapped->twist.twist.angular.x,
                         odomAftMapped->twist.twist.angular.y,
                         odomAftMapped->twist.twist.angular.z,

                         odomAftMapped->twist.twist.linear.x,
                         odomAftMapped->twist.twist.linear.y,
                         odomAftMapped->twist.twist.linear.z);

  //Save mapping degeneracy status
  _isMappingDegenerate = mapStatus->degenerate;
}

}  // end namespace loam
