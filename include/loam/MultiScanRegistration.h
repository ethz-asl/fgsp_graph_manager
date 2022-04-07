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

#ifndef LOAM_MULTISCANREGISTRATION_H
#define LOAM_MULTISCANREGISTRATION_H

#define PCL_NO_PRECOMPILE

#include "loam/ScanRegistration.h"
#include "math_utils.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

//CUSTOMIZATION
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
//CUSTOMIZATION

//Register Custom Velodyne and Ouster data types to use custom fields e.g. 'ring'
////////////////// VLP-16 Data structure support //////////////////
/** Velodyne Point struct */
struct PointVelodyne {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float intensity;                 // laser intensity reading
  uint16_t ring;                   // laser ring number
  float time;                      // laser time reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;
/** Register as pointcloud struct */
POINT_CLOUD_REGISTER_POINT_STRUCT (PointVelodyne,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (float, time, time)
)

////////////////// ANYbotics VLP-16 Data structure support //////////////////
/** ANYbotics Velodyne Point struct - difference in ring field type*/
struct PointVelodyneANYbotics
{
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float intensity;                 // laser intensity reading
  float ring;                      // laser ring number
  float time;                      // laser time reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;
/** Register as pointcloud struct */
POINT_CLOUD_REGISTER_POINT_STRUCT (PointVelodyneANYbotics,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, ring, ring)
                                  (float, time, time)
)

////////////////// OS0-128 Data structure support //////////////////
/** Ouster Point struct */
struct EIGEN_ALIGN16 PointOuster {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
/** Register as pointcloud struct for using */
POINT_CLOUD_REGISTER_POINT_STRUCT(PointOuster,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint32_t, t, t)
                                  (std::uint16_t, reflectivity, reflectivity)
                                  (std::uint8_t, ring, ring)
                                  (std::uint16_t, ambient, ambient)
                                  (std::uint32_t, range, range)
)
////////////////// OS0-128 Data structure support //////////////////

namespace loam{
/** \brief Class realizing a linear mapping from vertical point angle to the corresponding scan ring.
 *
 */
class MultiScanMapper {
 public:
  /** \brief Construct a new multi scan mapper instance.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  MultiScanMapper(const float &lowerBound = -15,
                  const float &upperBound = 15,
                  const uint16_t &nScanRings = 16);

  const float &getLowerBound() { return _lowerBound; }
  const float &getUpperBound() { return _upperBound; }
  const uint16_t &getNumberOfScanRings() { return _nScanRings; }

  /** \brief Set mapping parameters.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  void set(const float &lowerBound,
           const float &upperBound,
           const uint16_t &nScanRings);

  //CUSTOMIZATION
  void setLiDARName(const std::string &lidarName) { _lidarName = lidarName; }
  const std::string &getLiDARName() { return _lidarName; }
  //CUSTOMIZATION


  /** \brief Map the specified vertical point angle to its ring ID.
   *
   * @param angle the vertical point angle (in rad)
   * @return the ring ID
   */
  int getRingForAngle(const float &angle);

  /** Multi scan mapper for Velodyne VLP-16 according to data sheet. */
  static inline MultiScanMapper Velodyne_VLP_16() { return MultiScanMapper(-15, 15, 16); };

  /** Multi scan mapper for Velodyne HDL-32 according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_32() { return MultiScanMapper(-30.67f, 10.67f, 32); };

  /** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_64E() { return MultiScanMapper(-24.9f, 2, 64); };

  /** Multi scan mapper for Ouster O1-64 according to data sheet. */
  static inline MultiScanMapper Ouster_O1_64() { return MultiScanMapper(-16.611f, 16.611f, 64); };

  /** Multi scan mapper for Ouster O1-16 according to data sheet. */
  static inline MultiScanMapper Ouster_O1_16() { return MultiScanMapper(-16.611f, 16.611f, 16); };

  /** Multi scan mapper for RS-Bperl according to data sheet. */
  static inline MultiScanMapper RS_Bperl_32() { return MultiScanMapper(2.3125f, 89.5f, 32); };
  
  /** Multi scan mapper for Ouster O0-128 according to data sheet. */
  static inline MultiScanMapper Ouster_O0_128() { return MultiScanMapper(-46.2f, 46.2f, 128); };
  
  /** Multi scan mapper for Ouster O0-128-Div2 according to data sheet. */
  static inline MultiScanMapper Ouster_O0_128_Div2() { return MultiScanMapper(-46.2f, 46.2f, 128); };
  
private:
  std::string _lidarName; ///< CUSTOMIZATION LiDAR Name
  float _lowerBound;      ///< the vertical angle of the first scan ring
  float _upperBound;      ///< the vertical angle of the last scan ring
  uint16_t _nScanRings;   ///< number of scan rings
  float _factor;          ///< linear interpolation factor
};

/** \brief Class for registering point clouds received from multi-laser lidars.
 *
 */
class MultiScanRegistration : virtual public ScanRegistration {
 public:
  MultiScanRegistration(const MultiScanMapper &scanMapper = MultiScanMapper());  //smk:by default creates for VLP16

  bool setup(ros::NodeHandle &node, ros::NodeHandle &privateNode);

  /** \brief Handler method for input cloud messages.
   *
   * @param laserCloudMsg the new input cloud message to process
   */
  void handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

 private:
  /** \brief Setup component in active mode.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  bool setupROS(ros::NodeHandle &node, ros::NodeHandle &privateNode, RegistrationParams &config_out) override;

  /** \brief Process a new input cloud.
   *
   * @param laserCloudIn the new input cloud to process
   * @param scanTime the scan (message) timestamp
   */
  template <typename pointType>
  void process(const pcl::PointCloud<pointType> &laserCloudInput, const Time &scanTime);

private:
  int _systemDelay = 20;       ///< system startup delay counter
  MultiScanMapper _scanMapper; ///< mapper for mapping vertical point angles to scan ring IDs
  std::vector<pcl::PointCloud<pcl::PointXYZI>> _laserCloudScans;
  ros::Subscriber _subLaserCloud; ///< input cloud message subscriber

  //CUSTOMIZATION - Trasformation option if LiDAR is not mounted flat w.r.t robot body frame
  bool _rotateInputCloud = false;                         //Flag to rotate input cloud before estimating odometry/map so produced resuts are ROS frame aligned irrespective of LiDAR mounting orientation
  Eigen::Affine3d _affine = Eigen::Affine3d::Identity();  //Rotation applied to input cloud - ORDER YPR(radians)
  bool _useCloudIntensityandRingFields = false;           //Flag to use input pointcloud intensity or ring fields. Converts to custom PointXYZIR instead of PointXYZ
  float _minRange = 0.1, _maxRange = 80.0;                //Min/Max Range of useful points
  //CUSTOMIZATION
};

} // end namespace loam

#endif //LOAM_MULTISCANREGISTRATION_H
