#include "lidar_undistortion/velodyne_undistorter_ros.hpp"
#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <yaml-cpp/node/detail/node.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace lidar_undistortion;
using namespace velodyne_pointcloud;

VelodyneUndistorterROS::VelodyneUndistorterROS(ros::NodeHandle nh,
                                               ros::NodeHandle private_nh)
  : calibration_(data_.setup(private_nh)),
    fixed_frame_id_("odom"),
    lidar_frame_id_("velodyne"),
    tf_listener_(tf_buffer_)
{
if(calibration_)
{
  ROS_DEBUG_STREAM("Calibration file loaded.");
  config_.num_lasers = static_cast<uint16_t>(calibration_->num_lasers);
}
else
{
  ROS_ERROR_STREAM("Could not load calibration file!");
}

// advertise output point cloud (before subscribing to input data)
point_cloud_pub_ =
  nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

scan_sub_ = nh.subscribe("velodyne_packets", 10, &VelodyneUndistorterROS::scanCallback, this);

// Advertise the corrected pointcloud topic
corrected_pointcloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>(
      "pointcloud_corrected", 100, false);

private_nh.param("pose_topic", pose_topic_, pose_topic_);

pose_sub_ = nh.subscribe(pose_topic_, 100, &VelodyneUndistorterROS::poseCallback, this);

// Read the odom and lidar frame names from ROS params
private_nh.param("odom_frame_id", fixed_frame_id_, fixed_frame_id_);
private_nh.param("lidar_frame_id", lidar_frame_id_, lidar_frame_id_);
private_nh.param("base_frame_id", base_frame_id_, base_frame_id_);


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

void VelodyneUndistorterROS::scanCallback(const velodyne_msgs::VelodyneScan &scan_msg){

}

void VelodyneUndistorterROS::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_msg){
  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  tf2::fromMsg(pose_msg.pose.pose, pose);
  addPose(pose_msg.header.stamp.toNSec(), pose);
}

