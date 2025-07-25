#include "lidar_undistortion_ros/hesai_undistorter_ros.hpp"
#include <tf2/convert.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace lidar_undistortion;

HesaiUndistorterROS::HesaiUndistorterROS(rclcpp::Node& nh)
    : HesaiUndistorter(2e9), nh_(nh) {
  nh.get_parameter("point_cloud_input_topic", point_cloud_input_topic_);
  nh.get_parameter("pose_topic", pose_topic_);
  nh.get_parameter("point_cloud_output_topic", point_cloud_output_topic_);

  // Subscribe to the undistorted pointcloud topic
  pointcloud_sub_ =
      nh.create_subscription<sensor_msgs::msg::PointCloud2>(
          point_cloud_input_topic_, 100,
          std::bind(&HesaiUndistorterROS::pointcloudCallback, this, std::placeholders::_1));

  // Advertise the corrected pointcloud topic
  corrected_pointcloud_pub_ = nh.create_publisher<sensor_msgs::msg::PointCloud2>(
      point_cloud_output_topic_, 100);

  pose_sub_ =
      nh.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          pose_topic_, 100, std::bind(&HesaiUndistorterROS::poseCallback, this, std::placeholders::_1));
  
  // Read the odom and lidar frame names from ROS params
  if (!nh.get_parameter_or("fixed_frame_id", fixed_frame_id_, fixed_frame_id_)) {
    RCLCPP_WARN_STREAM(nh.get_logger(), "Could not read param \"fixed_frame_id\". "
                    << "Setting to default: " << fixed_frame_id_);
  }
  if (!nh.get_parameter_or("lidar_frame_id", lidar_frame_id_, lidar_frame_id_)) {
    RCLCPP_WARN_STREAM(nh.get_logger(), "Could not read param \"lidar_frame_id\". "
                    << "Setting to default: " << lidar_frame_id_);
  }
  if (!nh.get_parameter_or("base_frame_id", base_frame_id_, base_frame_id_)) {
    RCLCPP_WARN_STREAM(nh.get_logger(), "Could not read param \"base_frame_id\". "
                    << "Setting to default: " << base_frame_id_);
  }

  // retrieve the transform from base to lidar frame
  while (rclcpp::ok()) {
    try {
      geometry_msgs::msg::TransformStamped temp_transform;
      temp_transform = tf_buffer_->lookupTransform(
          base_frame_id_, lidar_frame_id_, rclcpp::Time(0));

      base_to_lidar_ = tf2::transformToEigen(temp_transform);
      break;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(nh.get_logger(), "%s", ex.what());
      sleep(1);
   }
  }

  RCLCPP_INFO(nh.get_logger(), "HesaiUndistorterROS ready.");
}

void HesaiUndistorterROS::pointcloudCallback(const sensor_msgs::msg::PointCloud2 &pointcloud_msg){
  HesaiCloud::Ptr pointcloud = std::make_shared<HesaiCloud>();
  pcl::fromROSMsg(pointcloud_msg, *pointcloud);
  if(!processCloud(pointcloud, pointcloud_msg.header.stamp.nanosec)){
    RCLCPP_INFO_STREAM(nh_.get_logger(), "processCloud for ns=" << pointcloud_msg.header.stamp.nanosec << " returned false, ignoring point cloud.");
    return;
  }
  sensor_msgs::msg::PointCloud2 pointcloud_corrected_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_corrected_msg);
  // Copy the pointcloud header correctly
  // NOTE: The header timestamp type in PCL pointclouds is narrower than in
  //       PointCloud2 msgs. We therefore copy this field directly from the
  //       losing timestamp accuracy.
  pointcloud_corrected_msg.header = pointcloud_msg.header;
  pointcloud_corrected_msg.header.frame_id = lidar_frame_id_;

  // Publish the corrected pointcloud
  corrected_pointcloud_pub_->publish(pointcloud_corrected_msg);
}

void HesaiUndistorterROS::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &pose_msg){
  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  tf2::fromMsg(pose_msg.pose.pose, pose);
  addPose(pose_msg.header.stamp.nanosec, pose);
  reprocessCloudBuffer();
}

bool HesaiUndistorterROS::processCloud(const HesaiCloud::Ptr& pointcloud,
                                       const uint64_t start_timestamp,
                                       const uint64_t desired_timestamp) {
  // Hesai pointclouds never have a fixed number of points

  times_lut_.clear();
  times_lut_.resize(pointcloud->points.size());

  double min_time = 0;
  minTimePoint(pointcloud, min_time);
  size_t idx = 0;
  for (const auto& p : pointcloud->points) {
    // convert time from hesai convention to our convention and add it to the
    // lookup table
    times_lut_[idx++] = static_cast<uint32_t>(
        static_cast<uint64_t>((p.timestamp - min_time) * 1'000'000'000));
  }
  return LidarUndistorter<PointHesai>::processCloud(pointcloud, start_timestamp,
                                                    desired_timestamp);
}

bool HesaiUndistorterROS::processCloud(const HesaiCloud::Ptr& pointcloud,
                          const uint64_t start_timestamp) {
  return processCloud(pointcloud, start_timestamp, start_timestamp);
}
void HesaiUndistorterROS::minTimePoint(const HesaiCloud::Ptr& cloud, double& min_time){

  //auto point_compare = ;
  min_time = (std::min_element(cloud->points.begin(), cloud->points.end(),
                               // lambda expression to compare the timestamp
                               // between two points
                               [](const PointHesai &a, const PointHesai &b) {
                                   return a.timestamp < b.timestamp;
                                 }))->timestamp;
}

void HesaiUndistorterROS::reprocessCloudBuffer(){
  // if there are point clouds in the buffer, we process them
  if(!cloud_history_.empty() && !odometry_history_.empty()){
    for(auto it = cloud_history_.begin(); it != cloud_history_.end(); ){
      DEBUG_PRINTLN("Processing cloud " << it->first);
      if(!processCloud(it->second, it->first)){
        if (it->first < odometry_history_.startTime()) {
          it = cloud_history_.erase(it);
        } else {
          ++it;
        }
      } else {
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*it->second, out_msg);
        out_msg.header.stamp = rclcpp::Time(it->first);
        out_msg.header.frame_id = lidar_frame_id_;
        corrected_pointcloud_pub_->publish(out_msg);
        DEBUG_PRINTLN("Processed. Cloud size is: " << cloud_history_.size());
        // the point cloud has been transformed successfully
        // we remove it from the buffer and return
        it = cloud_history_.erase(it);
      }
    }
  }
}
