#pragma once
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <velodyne_pointcloud/rawdata.h>
#include "lidar_undistortion/velodyne_point.hpp"
#include "lidar_undistortion/velodyne_container.hpp"
#include "lidar_undistortion/lidar_undistorter.hpp"
#include "lidar_undistortion/velodyne_rawdata.hpp"

namespace lidar_undistortion {

/// configuration parameters
 struct VelodyneUndistorterConfig
{
  double max_range = 100;          ///< maximum range to publish
  double min_range = 0;          ///< minimum range to publish
  uint16_t num_lasers = 16;       ///< number of lasers
};

class VelodyneUndistorterROS : public LidarUndistorter<velodyne::PointXYZIRT> {
public:
  using VelodyneCloud = pcl::PointCloud<velodyne::PointXYZIRT>;
  using VelodyneContainer = velodyne::VelodyneContainer<velodyne::PointXYZIRT>;
public:
  VelodyneUndistorterROS(ros::NodeHandle& nh,
                         ros::NodeHandle& private_nh);
  ~VelodyneUndistorterROS() override {
  }

  void scanCallback(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg);
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);
  void reprocessCloudBuffer() override;
private:
  ros::Subscriber scan_sub_;
  VelodyneUndistorterConfig config_;
  velodyne::RawData data_;
  boost::optional<velodyne_pointcloud::Calibration> calibration_;
  VelodyneContainer container_;              ///< input packet point cloud
  VelodyneCloud pointcloud_;

  std::string fixed_frame_id_ = "odom";
  std::string base_frame_id_ = "base";
  std::string pose_topic_ = "/state_estimator/pose_in_odom";
  // TF frame name of the lidar scan frame
  std::string lidar_frame_id_ = "velodyne";

  // ROS subscriber and publisher for the (un)corrected pointclouds
  ros::Subscriber pose_sub_;

  ros::Publisher corrected_pointcloud_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}
