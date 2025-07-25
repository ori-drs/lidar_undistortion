#include "lidar_undistortion_ros/velodyne_undistorter_ros.hpp"
#include <lidar_undistortion/velodyne_container.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/node/detail/node.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace lidar_undistortion;

VelodyneUndistorterROS::VelodyneUndistorterROS(rclcpp::Node& nh)
  : tf_listener_(tf_buffer_),
    velodyne_cvt_(nh)
{
  // set full 360 FoV and nominal range
  scan_sub_ = nh.subscribe("/velodyne_packets", 10, &VelodyneUndistorterROS::scanCallback, this);

  // Advertise the corrected pointcloud topic
  corrected_pointcloud_pub_ = nh.advertise<sensor_msgs::msg::PointCloud2>("pointcloud_corrected", 100, false);

  if(republish_original_cloud_){
    original_pointcloud_pub_ = nh.advertise<sensor_msgs::msg::PointCloud2>("pointcloud_original", 100, false);
  }

  if(!nh.param("pose_topic", pose_topic_, pose_topic_)){
        ROS_WARN("pose_topic not specified");
  }

  pose_sub_ = nh.subscribe(pose_topic_, 100, &VelodyneUndistorterROS::poseCallback, this);

  // Read the odom and lidar frame names from ROS params
  if(!nh.param("odom_frame_id", fixed_frame_id_, fixed_frame_id_)){
    ROS_WARN("odom_frame_id not specified");
  }
  if(!nh.param("lidar_frame_id", lidar_frame_id_, lidar_frame_id_)){
    ROS_WARN("lidar_frame_id not specified");
  }
  if(!nh.param("base_frame_id", base_frame_id_, base_frame_id_)){
    ROS_WARN("base_frame_id not specified");
  }


  // retrieve the transform from base to lidar frame
  while(nh.ok()){
    try{
      geometry_msgs::msg::TransformStamped temp_transform;
      temp_transform = tf_buffer_.lookupTransform(base_frame_id_, lidar_frame_id_,
                                                  ros::Time(0));

      base_to_lidar_ = tf2::transformToEigen(temp_transform);
      break;
    }
    catch (const tf2::TransformException& ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}

void VelodyneUndistorterROS::scanCallback(const velodyne_msgs::msg::VelodyneScan::ConstPtr &scan_msg){
  // // Create the corrected pointcloud ROS msg
  sensor_msgs::msg::PointCloud2 pointcloud_corrected_msg;
  VelodyneCloud::Ptr pc; // = boost::make_shared<VelodyneCloud>();

  velodyne_cvt_.scanToPointCloud(*scan_msg, *pc);
  ros::Time time_start = scan_msg->packets.front().stamp;

  if(republish_original_cloud_){
    sensor_msgs::msg::PointCloud2 pointcloud_original_msg;
    pcl::toROSMsg(*pc, pointcloud_original_msg);
    // Copy the pointcloud header correctly
    // NOTE: The header timestamp type in PCL pointclouds is narrower than in
    //       PointCloud2 msgs. We therefore copy this field directly from the
    //       losing timestamp accuracy.
    pointcloud_original_msg.header          = scan_msg->header;
    pointcloud_original_msg.header.stamp    = time_start;
    pointcloud_original_msg.header.frame_id = lidar_frame_id_;

    // publish the accumulated cloud message
    original_pointcloud_pub_.publish(pointcloud_original_msg);
  }

  if(!processCloud(pc, time_start.toNSec())){
    return;
  }

  pcl::toROSMsg(*pc, pointcloud_corrected_msg);
  // Copy the pointcloud header correctly
  // NOTE: The header timestamp type in PCL pointclouds is narrower than in
  //       PointCloud2 msgs. We therefore copy this field directly from the
  //       losing timestamp accuracy.
  pointcloud_corrected_msg.header          = scan_msg->header;
  pointcloud_corrected_msg.header.stamp    = time_start;
  pointcloud_corrected_msg.header.frame_id = lidar_frame_id_;

  // publish the accumulated cloud message
  corrected_pointcloud_pub_.publish(pointcloud_corrected_msg);
}

void VelodyneUndistorterROS::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &pose_msg){
//  ROS_WARN_STREAM("Got pose " << pose_msg.header.stamp.toNSec());
  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  tf2::fromMsg(pose_msg.pose.pose, pose);
  addPose(pose_msg.header.stamp.toNSec(), pose);
  reprocessCloudBuffer();
}

void VelodyneUndistorterROS::reprocessCloudBuffer(){
  // if there are point clouds in the buffer, we process them
  if(!cloud_history_.empty() && !odometry_history_.empty()){
    for(auto it = cloud_history_.lower_bound(odometry_history_.startTime()); it != cloud_history_.end(); ){

      DEBUG_PRINTLN("Processing cloud " << it->first);
      if(!processCloud(it->second, it->first)){
        ++it;
      } else {
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*it->second, out_msg);
        out_msg.header.stamp = ros::Time().fromNSec(it->first);
        out_msg.header.frame_id = lidar_frame_id_;
        corrected_pointcloud_pub_.publish(out_msg);
        // the point cloud has been transformed successfully
        // we remove it from the buffer and return
        it = cloud_history_.erase(it);
        DEBUG_PRINTLN("Processed. Cloud buffer size is: " << cloud_history_.size());

      }
    }
  }
}
