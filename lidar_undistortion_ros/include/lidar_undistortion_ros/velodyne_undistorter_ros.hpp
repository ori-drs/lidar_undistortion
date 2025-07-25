#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <lidar_undistortion/velodyne_point.hpp>
#include <lidar_undistortion/velodyne_container.hpp>
#include <lidar_undistortion/lidar_undistorter.hpp>
#include <lidar_undistortion/velodyne_rawdata.hpp>
#include <lidar_undistortion/ouster_point.hpp>
#include "lidar_undistortion_ros/velodyne_scan_converter.hpp"

namespace lidar_undistortion {

/// configuration parameters
 struct VelodyneUndistorterConfig
{
  double max_range = 100;          ///< maximum range to publish
  double min_range = 0;          ///< minimum range to publish
  uint16_t num_lasers = 16;       ///< number of lasers
};

class VelodyneUndistorterROS : public LidarUndistorter<PointOuster> {
public:
  using PointLidar = PointOuster;
  using VelodyneCloud = pcl::PointCloud<PointLidar>;
  using VelodyneContainer = velodyne::VelodyneContainer<PointLidar>;
  using ScanConverter = VelodyneScanConverter<PointLidar>;

public:
  VelodyneUndistorterROS(rclcpp::Node& nh);
  ~VelodyneUndistorterROS() override {
  }

  void scanCallback(const velodyne_msgs::msg::VelodyneScan::ConstPtr& scan_msg);
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg);
  void reprocessCloudBuffer() override;
private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber scan_sub_;
  ScanConverter velodyne_cvt_;
  VelodyneCloud pointcloud_;
  bool republish_original_cloud_ = true;

  std::string fixed_frame_id_ = "odom";
  std::string base_frame_id_ = "base";
  std::string pose_topic_ = "/state_estimator/pose_in_odom";
  // TF frame name of the lidar scan frame
  std::string lidar_frame_id_ = "velodyne";

  // ROS subscriber and publisher for the (un)corrected pointclouds
  ros::Subscriber pose_sub_;

  ros::Publisher corrected_pointcloud_pub_;
  ros::Publisher original_pointcloud_pub_;
};
}
