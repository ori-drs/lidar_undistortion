#pragma once
#include <Eigen/Dense>
#include <ouster_ros/point.h>
#include "lidar_undistortion/lidar_undistorter.hpp"
#include "lidar_undistortion/ouster_image_converter.hpp"

namespace lidar_undistortion {

class OusterUndistorter : public LidarUndistorter<ouster_ros::Point> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  using OusterPoint = ouster_ros::Point;
  using OusterCloud = pcl::PointCloud<OusterPoint>;

public:
  OusterUndistorter() : LidarUndistorter() {

  }

  OusterUndistorter(uint64_t pose_buffer_length) :
    LidarUndistorter(pose_buffer_length)
  {
    cvt_ = std::make_unique<OusterImageConverter>(1024, 64);
  }

  bool processCloud(const OusterCloud::Ptr& pointcloud,
                    const uint64_t timestamp) override;
protected:
  std::unique_ptr<OusterImageConverter> cvt_;
};
}
