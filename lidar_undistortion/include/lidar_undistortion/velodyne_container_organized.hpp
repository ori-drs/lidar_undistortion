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

#pragma once

#include "lidar_undistortion/velodyne_container_base.hpp"
#include "lidar_undistortion/velodyne_point.hpp"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


namespace velodyne {

class VelodyneContainerOrganized : public velodyne::VelodyneContainerBase {
public:
  using VelodynePoint = velodyne::PointXYZIRT;
  using VelodyneCloud = pcl::PointCloud<VelodynePoint>;

public:
  VelodyneContainerOrganized(const VelodyneContainerConfig& cfg)
  {
    setup(cfg);
  }

  VelodyneContainerOrganized() : VelodyneContainerOrganized(VelodyneContainerConfig()) {

  }

  VelodyneContainerOrganized(const double max_range,
                      const double min_range,
                      const unsigned int num_lasers,
                      const unsigned int scans_per_block) :
    VelodyneContainerOrganized(VelodyneContainerConfig(max_range, min_range, 1800, 16, false, scans_per_block))
  {

  }

  virtual void newLine();

  virtual void setup(const VelodyneContainerConfig& config);

  void addPoint(float x,
                float y,
                float z,
                uint16_t ring,
                uint16_t azimuth,
                float distance,
                float intensity,
                uint64_t time) override;

  virtual bool pointInRange(float distance) {
    return distance > 0;
  }

  VelodyneCloud::Ptr getCloud() override {
    //for(auto& idx : cloud_indices){
    //  std::cerr << "cloud_indices[" << i++ << "] = " << idx << std::endl;
    //}
    //std::cerr << "-------------------" << std::endl;
    cloud_indices = std::vector<size_t>(config_.init_height, 0);
    size_t offset_count = 0;
    for(auto it = azimuths.begin(); it != std::prev(azimuths.end()); ++it, ++offset_count)
    {
      if(*it > *std::next(it)){
        raw_offset = offset_count;
      }
    }

    azimuths.clear();

    return cloud;
  }

  size_t getPivot(){
    return raw_offset;
  }

private:
  VelodyneCloud::Ptr cloud;
  VelodynePoint pt;
  std::vector<size_t> cloud_indices;
  std::vector<uint16_t> azimuths;
  size_t raw_offset = 0;


  VelodyneContainerConfig config_;


};

} /* namespace velodyne_pointcloud */

