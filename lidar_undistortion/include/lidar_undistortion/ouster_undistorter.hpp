#pragma once
#include <Eigen/Dense>
#include <ouster_ros/point_os1.h>
#include "lidar_undistortion/lidar_undistorter.hpp"

namespace lidar_undistortion {

class OusterUndistorter : public LidarUndistorter<ouster_ros::OS1::PointOS1> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  using OusterPoint = ouster_ros::OS1::PointOS1;
  using OusterCloud = pcl::PointCloud<OusterPoint>;

public:
  OusterUndistorter() : LidarUndistorter() {

  }

  OusterUndistorter(uint64_t pose_buffer_length) :
    LidarUndistorter(pose_buffer_length)
  {
  }

  bool processCloud(const OusterCloud::Ptr& pointcloud,
                    const uint64_t timestamp) override;

};
}
