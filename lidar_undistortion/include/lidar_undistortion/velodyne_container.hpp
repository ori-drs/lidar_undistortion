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
#include <boost/make_shared.hpp>
#include <string>


namespace velodyne {

template <class PointT>
class VelodyneContainer : public VelodyneContainerBase<PointT> {

  using VelodyneCloud = typename VelodyneContainerBase<PointT>::VelodyneCloud;

public:
  VelodyneContainer(const VelodyneContainerConfig& cfg) :
    pc(boost::make_shared<VelodyneCloud>()),
    VelodyneContainerBase<PointT>(cfg)
  {

  }

  VelodyneContainer() : VelodyneContainer(VelodyneContainerConfig()) {

  }

  VelodyneContainer(const double max_range,
                    const double min_range,
                    const unsigned int num_lasers,
                    const unsigned int scans_per_block) :
    VelodyneContainer(VelodyneContainerConfig(max_range, min_range, 1800, 16, false, scans_per_block))
  {

  }

  ~VelodyneContainer() override {

  }

  void addPoint(float x,
                float y,
                float z,
                uint16_t ring,
                uint16_t azimuth,
                float distance,
                float intensity,
                uint64_t time) override;

  bool pointInRange(float distance) const override {
    return distance > 0;
  }

  typename VelodyneCloud::Ptr getCloud() {
    return pc;
  }

private:
  typename VelodyneCloud::Ptr pc;

};

template <class PointT>
void VelodyneContainer<PointT>::addPoint(float x,
                                         float y,
                                         float z,
                                         uint16_t ring,
                                         uint16_t /*azimuth*/,
                                         float /*distance*/,
                                         float intensity,
                                         uint64_t time)
{
  // convert polar coordinates to Euclidean XYZ
  PointT point;
  point.ring = ring;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  //point.time = time;

  // append this point to the cloud
  pc->points.push_back(point);
  ++pc->width;
}
} /* namespace velodyne */

