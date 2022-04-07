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

#ifndef LOAM_LASERODOMETRY_H
#define LOAM_LASERODOMETRY_H

#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "BasicLaserOdometry.h"
#include "Twist.h"
#include "nanoflann_pcl.h"
#include "math_utils.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

namespace loam {

/** \brief Implementation of the LOAM laser odometry component.
 *
 */
class LaserOdometry : public BasicLaserOdometry {
 public:
  explicit LaserOdometry(float scanPeriod = 0.1, uint16_t ioRatio = 2, size_t maxIterations = 25);

  /** \brief Setup component.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  virtual bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  /** \brief Handler method for a new sharp corner cloud.
   *
   * @param cornerPointsSharpMsg the new sharp corner cloud message
   */
  void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg);

  /** \brief Handler method for a new less sharp corner cloud.
   *
   * @param cornerPointsLessSharpMsg the new less sharp corner cloud message
   */
  void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg);

  /** \brief Handler method for a new flat surface cloud.
   *
   * @param surfPointsFlatMsg the new flat surface cloud message
   */
  void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg);

  /** \brief Handler method for a new less flat surface cloud.
   *
   * @param surfPointsLessFlatMsg the new less flat surface cloud message
   */
  void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg);

  /** \brief Handler method for a new full resolution cloud.
   *
   * @param laserCloudFullResMsg the new full resolution cloud message
   */
  void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

  /** \brief Handler method for a new IMU transformation information.
   *
   * @param laserCloudFullResMsg the new IMU transformation information message
   */
  void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg);

  /** \brief Try to process buffered data. */
  void process();

  //CUSTOMIZATION
  /** \brief IMU Callback Function for handling incoming IMU messages
     *
     * @param imu_ptr pointer to incoming ROS IMU message
  */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr);

  /** \brief Sync Callback Handler for all input clouds
   * @param pclSharp Sharp Corner Points
   * @param pclLessSharp Less Sharp Corner Points
   * @param pclFlat Flat Points
   * @param pclLessFlat Less Flat Points
   * @param pclFull Full PCL arranged ring-wise
   */
  void syncCloudsHandler(const sensor_msgs::PointCloud2ConstPtr& pclSharp,
                         const sensor_msgs::PointCloud2ConstPtr& pclLessSharp,
                         const sensor_msgs::PointCloud2ConstPtr& pclFlat,
                         const sensor_msgs::PointCloud2ConstPtr& pclLessFlat,
                         const sensor_msgs::PointCloud2ConstPtr& pclFull);
  //CUSTOMIZATION

 protected:
  /** \brief Publish the current result via the respective topics. */
  void publishResult();

 private:
  uint16_t _ioRatio;  ///< ratio of input to output frames

  ros::Time _timeCornerPointsSharp;      ///< time of current sharp corner cloud
  ros::Time _timeCornerPointsLessSharp;  ///< time of current less sharp corner cloud
  ros::Time _timeSurfPointsFlat;         ///< time of current flat surface cloud
  ros::Time _timeSurfPointsLessFlat;     ///< time of current less flat surface cloud
  ros::Time _timeLaserCloudFullRes;      ///< time of current full resolution cloud
  ros::Time _timeImuTrans;               ///< time of current IMU transformation information

  tf::StampedTransform _laserOdometryTrans;  ///< laser odometry transformation

  ros::Publisher _pubLaserCloudCornerLast;  ///< last corner cloud message publisher
  ros::Publisher _pubLaserCloudSurfLast;    ///< last surface cloud message publisher
  ros::Publisher _pubLaserCloudFullRes;     ///< full resolution cloud message publisher
  ros::Publisher _pubLaserOdometry;         ///< laser odometry publisher
  tf::TransformBroadcaster _tfBroadcaster;  ///< laser odometry transform broadcaster

  ros::Subscriber _subImuTrans;  ///< IMU transformation information message subscriber
  ros::Subscriber _subImu;       ///< IMU subscriber

  // ARL Coordinate Frame Facilitators
  tf::StampedTransform _laserOdometryTrans_CORRECTED;  ///< laser odometry transformation CORRECTED
  ros::Publisher _pubLaserOdometry_CORRECTED;          ///< laser odometry message CORRECTED publisher
  ros::Publisher _pubLaserImuBias;                     ///< laser odometry imu bias publisher
  // END ARL Coordinate Frame Facilitators

  //CUSTOMIZATION
  ros::Publisher _pubOptimizationStatus;                                // Publisher for advertising the status of laser odomertry optimzation
  ros::Publisher _pubLaserOdometryStatus;                               // Publisher for advertising current condtion of Laser Odomertry Degeneracy
  message_filters::Subscriber<sensor_msgs::PointCloud2> _subSharp;      // Sharp corner cloud message subscriber
  message_filters::Subscriber<sensor_msgs::PointCloud2> _subLessSharp;  // Less sharp corner cloud message subscriber
  message_filters::Subscriber<sensor_msgs::PointCloud2> _subFlat;       // Flat surface cloud message subscriber
  message_filters::Subscriber<sensor_msgs::PointCloud2> _subLessFlat;   // Less flat surface cloud message subscriber
  message_filters::Subscriber<sensor_msgs::PointCloud2> _subFull;       // Full resolution cloud message subscriber
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
                                                    sensor_msgs::PointCloud2,
                                                    sensor_msgs::PointCloud2,
                                                    sensor_msgs::PointCloud2,
                                                    sensor_msgs::PointCloud2>
      _pclExactSyncPolicy;                                                                 // ROS Exact Sync Policy Object
  boost::shared_ptr<message_filters::Synchronizer<_pclExactSyncPolicy>> _pclExactSyncPtr;  // ROS Exact Sync Policy Message Filter
  tf::StampedTransform _cameraInitGravityAlignedTf;                                        // Transform from camera_init_CORRECTED to Gravity Aligned Frame
  std::string _cameraFrameId;                                                              // Frame id of camera
  //CUSTOMIZATION
};

}  // end namespace loam

#endif  //LOAM_LASERODOMETRY_H
