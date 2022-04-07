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

#ifndef LOAM_TRANSFORMMAINTENANCE_H
#define LOAM_TRANSFORMMAINTENANCE_H

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include "loam/BasicTransformMaintenance.h"
#include "loam/OptStatus.h"

namespace loam {

/** \brief Implementation of the LOAM transformation maintenance component.
 *
 */
class TransformMaintenance : public BasicTransformMaintenance {
 public:
  TransformMaintenance();

  /** \brief Setup component.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  virtual bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  /** \brief Handler method for laser odometry and status messages.
   *
   * @param laserOdometry the new laser odometry
   * @param laserStatus status of laser odometry
   */
  void syncLaserOdomHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry, const OptStatusConstPtr& laserStatus);

  /** \brief Handler method for mapping odometry and statusmessages.
   *
   * @param odomAftMapped the new mapping odometry
   * @param mapStatus status of the new mapping odometry
   */
  void syncMapOdomHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped, const OptStatusConstPtr& mapStatus);

 private:
  nav_msgs::Odometry _laserOdometry2;         ///< latest integrated laser odometry message
  tf::StampedTransform _laserOdometryTrans2;  ///< latest integrated laser odometry transformation

  ros::Publisher _pubLaserOdometry2;         ///< integrated laser odometry publisher
  tf::TransformBroadcaster _tfBroadcaster2;  ///< integrated laser odometry transformation broadcaster

  //Synced subscriber
  typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, OptStatus> _syncPolicy;  // ROS Sync Policy Object
  //Scan-to-Scan Odometry - High frequency
  message_filters::Subscriber<nav_msgs::Odometry> _subLaserOdom;
  message_filters::Subscriber<OptStatus> _subLaserOdomStatus;
  boost::shared_ptr<message_filters::Synchronizer<_syncPolicy>> _laserSyncPtr;  // ROS Sync Policy Message Filter
  //Scan-to-Map Odometry - Low frequency
  message_filters::Subscriber<nav_msgs::Odometry> _subMapOdom;
  message_filters::Subscriber<OptStatus> _subMapOdomStatus;
  boost::shared_ptr<message_filters::Synchronizer<_syncPolicy>> _mapSyncPtr;  // ROS Sync Policy Message Filter

  bool _isMappingDegenerate = false;  //Flag to store status of scan-to-map alignment
  ros::Publisher _pubOdomStatus;

  // ARL Coordinate Frame Facilitators
  nav_msgs::Odometry _laserOdometry2_CORRECTED;                          ///< latest integrated laser odometry message CORRECTED
  geometry_msgs::TransformStamped _gm_laserTransformStamped2_CORRECTED;  ///< geometry_msg of latest integrated laser odometry message CORRECTED

  ros::Publisher _pubLaserOdometry2_CORRECTED;          ///< integrated laser odometry CORRECTED publisher
  ros::Publisher _pubLaserTransformStamped2_CORRECTED;  ///< integrated laser transform CORRECTED publisher
  // END ARL Coordinate Frame Facilitators

  //CUSTOMIZATION
  ros::Publisher _pubPath;  // ROS path publisher for integrated odometry
  nav_msgs::Path _pathMsg;  // ROS path message for integrated odometry
  //CUSTOMIZATION
};

}  // end namespace loam

#endif  //LOAM_TRANSFORMMAINTENANCE_H
