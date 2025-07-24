#pragma once
#include <lidar_undistortion/ouster_undistorter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <lidar_undistortion_msgs/RangeImage.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


namespace lidar_undistortion {

class OusterUndistorterROS : public OusterUndistorter {
public:
 OusterUndistorterROS(rclcpp::Node& nh);
 ~OusterUndistorterROS() override = default;

 void pointcloudCallback(const sensor_msgs::PointCloud2& pointcloud_msg);
 void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);

 // this re-implementation is identical in all ROS implementations, but
 // an abstraction would lead to the Inheritance Diamond Problem so we leave it
 void reprocessCloudBuffer() override;
private:
 // TF frame name of a frame that can be considered fixed
 // NOTE: When correcting the pointcloud distortion, each point is first
 //       transformed into a fixed frame (F), using the lidar's true pose
 //       at the time that the point was recorded (S_correct).
 //       The point is then transformed back into the scan frame (S_original)
 //       matching the pointcloud message's frame_id and timestamp.
 std::string fixed_frame_id_;
 std::string base_frame_id_ = "imu";
 std::string pose_topic_;
 std::string point_cloud_input_topic_ = "/os_cloud_node/points";
 std::string point_cloud_output_topic_ = "/os_cloud_node/points_corrected";
 // TF frame name of the lidar scan frame
 std::string lidar_frame_id_;

 // ROS subscriber and publisher for the (un)corrected pointclouds
 ros::Subscriber pointcloud_sub_;
 ros::Subscriber pose_sub_;

 ros::Publisher corrected_pointcloud_pub_;
 image_transport::ImageTransport img_transp_;
 image_transport::Publisher corrected_range_pub_;
 sensor_msgs::ImagePtr range_img_msg_;

 tf2_ros::Buffer tf_buffer_;
 tf2_ros::TransformListener tf_listener_;

protected:
  std::unique_ptr<OusterImageConverter> cvt_;
  cv::Mat ranges_;
  cv::Mat altitudes_;
  cv::Mat azimuths_;
  cv::Mat ranges_viz_;
  cv::Mat intensities_;
  cv::Mat reflectivities_;
};
}
