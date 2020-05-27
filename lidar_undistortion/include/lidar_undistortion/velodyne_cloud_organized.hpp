// Copyright (C) 2012, 2019 Austin Robot Technology, Jack O'Quin, Joshua Whitley, Sebastian Pütz
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef VELODYNE_POINTCLOUD_ORGANIZED_CLOUDXYZIR_H
#define VELODYNE_POINTCLOUD_ORGANIZED_CLOUDXYZIR_H

#include <velodyne_pointcloud/datacontainerbase.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <pcl/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>


namespace velodyne_pointcloud {

struct OrganizedCloudConfig
{
  double max_range;          ///< maximum range to publish
  double min_range;          ///< minimum range to publish
  unsigned int init_width;
  unsigned int init_height;
  bool is_dense;
  unsigned int scans_per_packet;

  OrganizedCloudConfig() = default;

  OrganizedCloudConfig(double max_range,
                       double min_range,
                       unsigned int init_width,
                       unsigned int init_height,
                       bool is_dense,
                       unsigned int scans_per_packet)
    : max_range(max_range),
      min_range(min_range),
      init_width(init_width),
      init_height(init_height),
      is_dense(is_dense),
      scans_per_packet(scans_per_packet)
  {
    /*ROS_INFO_STREAM("Initialized container with "
                    << "min_range: " << min_range << ", max_range: " << max_range
                    << ", target_frame: " << target_frame << ", fixed_frame: " << fixed_frame
                    << ", init_width: " << init_width << ", init_height: " << init_height
                    << ", is_dense: " << is_dense << ", scans_per_packet: " << scans_per_packet);
  */}
};

class OrganizedCloudXYZIR : public velodyne_rawdata::DataContainerBase
{
public:
  // using VelodyneCloud = pcl::PointCloud<velodyne_pointcloud::PointXYZIR>;
  using VelodyneCloud = sensor_msgs::PointCloud2;

public:
  OrganizedCloudXYZIR() = default;

  OrganizedCloudXYZIR(const OrganizedCloudConfig& cfg) :
    iter_x(cloud, "x"),
    iter_y(cloud, "y"),
    iter_z(cloud, "z"),
    iter_intensity(cloud, "intensity"),
    iter_ring(cloud, "ring"),
    config_(cfg)
  {

  }

  OrganizedCloudXYZIR(const double max_range,
                      const double min_range,
                      const unsigned int num_lasers,
                      const unsigned int scans_per_block) :
    OrganizedCloudXYZIR(OrganizedCloudConfig(max_range, min_range, 1800, 16, false, scans_per_block))
  {

  }

  virtual void newLine();

  virtual void setup(const OrganizedCloudConfig& config);

  void addPoint(const float& x,
                const float& y,
                const float& z,
                const uint16_t& ring,
                const uint16_t& azimuth,
                const float& distance,
                const float& intensity) override;

  virtual bool pointInRange(float distance) {
    return distance > 0;
  }

private:
  VelodyneCloud cloud;
  sensor_msgs::PointCloud2Iterator<float> iter_x;
  sensor_msgs::PointCloud2Iterator<float> iter_y;
  sensor_msgs::PointCloud2Iterator<float> iter_z;
  sensor_msgs::PointCloud2Iterator<float> iter_intensity;
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring;

  OrganizedCloudConfig config_;


};
} /* namespace velodyne_pointcloud */
#endif  // VELODYNE_POINTCLOUD_ORGANIZED_CLOUDXYZIR_H

