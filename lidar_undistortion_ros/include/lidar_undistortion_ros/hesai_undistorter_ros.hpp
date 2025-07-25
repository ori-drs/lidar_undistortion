#pragma once
#include "lidar_undistortion/hesai_point.hpp"
#include "lidar_undistortion/lidar_undistorter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>

namespace lidar_undistortion {

class HesaiUndistorterROS : public LidarUndistorter<PointHesai> {
public:
  using HesaiCloud = pcl::PointCloud<PointHesai>;
  using HesaiUndistorter = LidarUndistorter<PointHesai>;
public:
 HesaiUndistorterROS(rclcpp::Node& nh);
 ~HesaiUndistorterROS() override = default;
 void pointcloudCallback(const sensor_msgs::msg::PointCloud2& pointcloud_msg);
 // copied from OusterUndistorterROS
 // TODO move into a common class
 void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);

 bool processCloud(const HesaiCloud::Ptr& pointcloud,
                   const uint64_t start_timestamp,
                   const uint64_t desired_timestamp) override;

 // copied from OusterUndistorterROS
 // TODO move into a common class
 bool processCloud(const HesaiCloud::Ptr& pointcloud,
                   const uint64_t start_timestamp) override;

 // this re-implementation is identical in all ROS implementations, but
 // an abstraction would lead to the Inheritance Diamond Problem so we leave it
 // TODO reconsider making a LidarUndistortionROS class and solve the IDP.
 void reprocessCloudBuffer() override;
private:
 std::string lidar_frame_id_ = "pandar";
 std::string fixed_frame_id_ = "odom";
 std::string base_frame_id_ = "imu";
 std::string point_cloud_input_topic_ = "/hesai/pandar";
 std::string point_cloud_output_topic_ = "/hesai/pandar_corrected";
 std::string pose_topic_;
 ros::Publisher corrected_pointcloud_pub_;
 tf2_ros::TransformListener tf_listener_;
 tf2_ros::Buffer tf_buffer_;
 // ROS subscriber and publisher for the (un)corrected pointclouds
 ros::Subscriber pointcloud_sub_;
 ros::Subscriber pose_sub_;

 void minTimePoint(const HesaiCloud::Ptr& cloud, double& min_time);


};


}
