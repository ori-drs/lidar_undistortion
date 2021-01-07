#pragma once
#include <Eigen/Dense>
#include "lidar_undistortion/lidar_undistorter.hpp"
#include "lidar_undistortion/ouster_image_converter.hpp"
#include "lidar_undistortion/ouster_point.hpp"

namespace lidar_undistortion {

class OusterUndistorter : public LidarUndistorter<PointOuster> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  using OusterCloud = pcl::PointCloud<PointOuster>;

public:
  OusterUndistorter() : LidarUndistorter() {

  }

  OusterUndistorter(uint64_t pose_buffer_length) :
    LidarUndistorter(pose_buffer_length)
  {

  }

  bool processCloud(const OusterCloud::Ptr& pointcloud,
                    const uint64_t timestamp) override;

  bool processCloud(const OusterCloud::Ptr& pointcloud,
                    const uint64_t timestamp,
                    const uint64_t desired_timestamp) override;
};
}
