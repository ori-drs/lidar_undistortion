#ifndef LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_
#define LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_

#include <Eigen/Eigen>
#include <string>
#include <pcl/point_cloud.h>
#include <ouster_ros/point_os1.h>
#include "lidar_undistortion/pose_buffer.hpp"

namespace lidar_undistortion {

class LidarUndistorter {
public:
  const uint64_t time_offset = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using OusterCloud = pcl::PointCloud<ouster_ros::OS1::PointOS1>;
  using CloudHistory = std::map<uint64_t, OusterCloud::Ptr>;
  using PosePair = std::pair<uint64_t, Eigen::Isometry3d>;
  using Vector3d = PoseBuffer::Vector3d;
  using Quaternion = PoseBuffer::Quaternion;

protected:
  virtual void addPose(uint64_t nsec, Eigen::Isometry3d& pose);

  virtual bool processCloud(const OusterCloud::Ptr& pointcloud,
                            const uint64_t timestamp);

protected:
  Eigen::Isometry3d base_to_lidar_ = Eigen::Isometry3d::Identity();
  CloudHistory cloud_history_;
  PoseBuffer odometry_history_;
};




}  // namespace lidar_undistortion

#endif  // LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_
