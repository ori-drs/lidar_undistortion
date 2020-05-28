#pragma once
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

#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Dense>
#include <string>
#include <algorithm>
#include <cstdarg>

namespace velodyne {

struct VelodyneContainerConfig {
  double max_range;          ///< maximum range to publish
  double min_range;          ///< minimum range to publish
  unsigned int init_width;
  unsigned int init_height;
  bool is_dense;
  unsigned int scans_per_packet;

  VelodyneContainerConfig() = default;

  VelodyneContainerConfig(double max_range,
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
  }
};

class VelodyneContainerBase {
public:
  VelodyneContainerBase() = default;
  virtual ~VelodyneContainerBase() = default;


  virtual void setup(const VelodyneContainerConfig& config) {
    config_ = config;
  }

  virtual void addPoint(float x,
                        float y,
                        float z,
                        const uint16_t ring,
                        const uint16_t azimuth,
                        const float distance,
                        const float intensity,
                        const uint64_t time) = 0;

  virtual void newLine() {

  }


  virtual bool pointInRange(float range) const {
    return (range >= config_.min_range && range <= config_.max_range);
  }

protected:
  VelodyneContainerConfig config_;
};

}
