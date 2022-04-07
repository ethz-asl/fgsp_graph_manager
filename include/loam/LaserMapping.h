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

#ifndef LOAM_LASERMAPPING_H
#define LOAM_LASERMAPPING_H

#include "BasicLaserMapping.h"
#include "common.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

namespace loam {
/** \brief Implementation of the LOAM laser mapping component.
 *
 */
class LaserMapping : public BasicLaserMapping {
 public:
  explicit LaserMapping(const float& scanPeriod = 0.1, const size_t& maxIterations = 10);  //smk:Original

  /** \brief Setup component in active mode.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  virtual bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  /** \brief Handler method for a new last corner cloud.
   *
   * @param cornerPointsLastMsg the new last corner cloud message
   */
  void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg);

  /** \brief Handler method for a new last surface cloud.
   *
   * @param surfacePointsLastMsg the new last surface cloud message
   */
  void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg);

  /** \brief Handler method for a new full resolution cloud.
   *
   * @param laserCloudFullResMsg the new full resolution cloud message
   */
  void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

  /** \brief Handler method for a new laser odometry.
   *
   * @param laserOdometry the new laser odometry message
   */
  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);

  /** \brief Handler method for IMU messages.
   *
   * @param imuIn the new IMU message
   */
  void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn);

  /** \brief Try to process buffered data. */
  void process();

  /** \brief Extract relevant static TF for VIO-LOAM integration */
  void extractStaticTFForVIOIntegration();

  /** \brief Extract relevant static to publish TF for external odometry estimator */
  void extractStaticTFForExternalOdometryFrame();

  //CUSTOMIZATION
  /** \brief Handler for degeneracy status of laser odometry. */
  void laserOdometryStatusHandler(const std_msgs::HeaderConstPtr& statusPtr);

  /** \brief Service call to save LOAM Corner and Suface submaps of initial/center sector. */
  bool saveSubMapsToFile(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/);

  /** \brief Sync Callback Handler for all input clouds and odometry
   * @param pclCorner Sharp Corner Points
   * @param pclFlat Flat Points
   * @param pclFull Full PCL arranged ring-wise
   * @param laserOdom scan-to-scan Laser Odometry input
   */
  void syncInputsHandler(const sensor_msgs::PointCloud2ConstPtr& pclCorner,
                         const sensor_msgs::PointCloud2ConstPtr& pclFlat,
                         const sensor_msgs::PointCloud2ConstPtr& pclFull,
                         const nav_msgs::OdometryConstPtr& laserOdom);
  //CUSTOMIZATION

 protected:
  /** \brief Publish the current result via the respective topics. */
  void publishResult();

 private:
  ros::Time _timeLaserCloudCornerLast;  ///< time of current last corner cloud
  ros::Time _timeLaserCloudSurfLast;    ///< time of current last surface cloud
  ros::Time _timeLaserCloudFullRes;     ///< time of current full resolution cloud
  ros::Time _timeLaserOdometry;         ///< time of current laser odometry

  tf::StampedTransform _aftMappedTrans;  ///< mapping odometry transformation

  ros::Publisher _pubLaserCloudSurround;    ///< map cloud message publisher
  ros::Publisher _pubLaserCloudFullRes;     ///< current full resolution cloud message publisher
  ros::Publisher _pubOdomAftMapped;         ///< mapping odometry publisher
  tf::TransformBroadcaster _tfBroadcaster;  ///< mapping odometry transform broadcaster

  ros::Subscriber _subImu;  ///< IMU message subscriber

  message_filters::Subscriber<sensor_msgs::PointCloud2> _subCornerCloud;  // Last corner cloud message subscriber
  message_filters::Subscriber<sensor_msgs::PointCloud2> _subFlatCloud;    // Last surface cloud message subscriber
  message_filters::Subscriber<sensor_msgs::PointCloud2> _subFullCloud;    // Full resolution cloud message subscriber
  message_filters::Subscriber<nav_msgs::Odometry> _subLaserOdom;          // Laser odometry message subscriber
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
                                                    sensor_msgs::PointCloud2,
                                                    sensor_msgs::PointCloud2,
                                                    nav_msgs::Odometry>
      _pclExactSyncPolicy;                                                                 // ROS Exact Sync Policy Object
  boost::shared_ptr<message_filters::Synchronizer<_pclExactSyncPolicy>> _pclExactSyncPtr;  // ROS Exact Sync Policy Message Filter

  // ARL Coordinate Frame Facilitators
  geometry_msgs::TransformStamped _gm_aftMappedTransformStamped_CORRECTED;  ///< geometry_msg of mapping odometry message CORRECTED

  ros::Publisher _pubOdomAftMapped_CORRECTED;              ///< mapping odometry message CORRECTED
  ros::Publisher _pubAftMappedTransformStamped_CORRECTED;  ///< mapping transform CORRECTED
  ros::Publisher _pubOptimizationStatus;
  // END ARL Coordinate Frame Facilitators

  //CUSTOMIZATION
  ros::Subscriber _subLaserOdometryStatus;                        // Laser odometry status subscriber
  ros::ServiceServer _srvSaveSubmapsToFile;                       // saves corner and surface feature submaps to file
  ros::Publisher _pubCurrentCornerSubmap, _pubCurrentSurfSubmap;  // ROS publisher for current corner/flat submaps
  ros::Publisher _pubSavedCornerSubmap, _pubSavedSurfSubmap;      // ROS publisher for loaded saved corner/flat submaps
  ros::Publisher _pubPath;                                        // ROS path publisher for mapping odometry
  nav_msgs::Path _pathMsg;                                        // ROS path message for mapping odometry
  bool _forcePathPublishing = false;                              // Force ROS path message publishing for mapping odometry
  ros::Publisher _pubMap;                                         // ROS publisher for Full map
  int _mapPublishRateDivider = 20;                                // Publishes full map at 1.0/(scanPeriod * ioRatio * mapPublishRateDivider)
  int _mapCounter = 0;                                            // Counter for full map publishing
  ros::Publisher _pubSubMapSearchCloud;                           // ROS publisher for current cloud to be searched in the submap
  //CUSTOMIZATION
};

}  // end namespace loam

#endif  //LOAM_LASERMAPPING_H
