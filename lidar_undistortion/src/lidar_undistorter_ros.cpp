#include "lidar_undistortion/lidar_undistorter_ros.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace lidar_undistortion;

LidarUndistorterROS::LidarUndistorterROS(ros::NodeHandle nh,
                                         ros::NodeHandle nh_private)
  : fixed_frame_id_("odom"),
    lidar_frame_id_("os1_lidar"),
    tf_listener_(tf_buffer_)
{
  // Subscribe to the undistorted pointcloud topic
  pointcloud_sub_ = nh.subscribe("pointcloud", 100,
                                 &LidarUndistorterROS::pointcloudCallback, this);

  // Advertise the corrected pointcloud topic
  corrected_pointcloud_pub_ = nh_private.advertise<sensor_msgs::PointCloud2>(
        "pointcloud_corrected", 100, false);

  nh_private.param("pose_topic", pose_topic_, pose_topic_);

  pose_sub_ = nh.subscribe(pose_topic_, 100, &LidarUndistorterROS::poseCallback, this);

  // Read the odom and lidar frame names from ROS params
  nh_private.param("odom_frame_id", fixed_frame_id_, fixed_frame_id_);
  nh_private.param("lidar_frame_id", lidar_frame_id_, lidar_frame_id_);
  nh_private.param("base_frame_id", base_frame_id_, base_frame_id_);


  // retrieve the transform from base to lidar frame
  while(nh.ok()){
    try{
      geometry_msgs::TransformStamped temp_transform;
      temp_transform = tf_buffer_.lookupTransform(base_frame_id_, lidar_frame_id_,
                                                  ros::Time(0));

      base_to_lidar_ = tf2::transformToEigen(temp_transform);
      break;
    }
    catch (tf2::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}

void LidarUndistorterROS::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg) {
  // Convert the pointcloud to PCL
  OusterCloud::Ptr pointcloud = boost::make_shared<OusterCloud>();
  pcl::fromROSMsg(*pointcloud_msg, *pointcloud);

  if(!processCloud(pointcloud, pointcloud_msg->header.stamp.toNSec())){
    return;
  };

  // Create the corrected pointcloud ROS msg
  sensor_msgs::PointCloud2 pointcloud_corrected_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_corrected_msg);

  // Copy the pointcloud header correctly
  // NOTE: The header timestamp type in PCL pointclouds is narrower than in
  //       PointCloud2 msgs. We therefore copy this field directly from the
  //       losing timestamp accuracy.
  pointcloud_corrected_msg.header = pointcloud_msg->header;

  // Publish the corrected pointcloud
  corrected_pointcloud_pub_.publish(pointcloud_corrected_msg);
}

void LidarUndistorterROS::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_msg){
  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  tf2::fromMsg(pose_msg.pose.pose, pose);
  addPose(pose_msg.header.stamp.toNSec(), pose);
}

