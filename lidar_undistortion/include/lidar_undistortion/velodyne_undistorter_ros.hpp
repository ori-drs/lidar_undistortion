#pragma once
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include "lidar_undistorter.hpp"

namespace lidar_undistortion {

/// configuration parameters
 struct VelodyneUndistorterConfig
{
  double max_range = 100;          ///< maximum range to publish
  double min_range = 0;          ///< minimum range to publish
  uint16_t num_lasers = 16;       ///< number of lasers
};

class VelodyneUndistorterROS : public LidarUndistorter<velodyne_rawdata::VPoint> {
public:
  VelodyneUndistorterROS(ros::NodeHandle nh,
                         ros::NodeHandle private_nh);
  ~VelodyneUndistorterROS()
  {
  }

  void scanCallback(const velodyne_msgs::VelodyneScan& scan_msg);
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);
private:
  ros::Subscriber scan_sub_;
  ros::Publisher point_cloud_pub_;
  VelodyneUndistorterConfig config_;
  boost::optional<velodyne_pointcloud::Calibration> calibration_;
  velodyne_pointcloud::PointcloudXYZIR container_;              ///< input packet point cloud
  velodyne_rawdata::VPointCloud pointcloud_;
  velodyne_rawdata::RawData data_;

  std::string fixed_frame_id_;
  std::string base_frame_id_ = "imu";
  std::string pose_topic_;
  // TF frame name of the lidar scan frame
  std::string lidar_frame_id_;

  // ROS subscriber and publisher for the (un)corrected pointclouds
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber pose_sub_;

  ros::Publisher corrected_pointcloud_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}
