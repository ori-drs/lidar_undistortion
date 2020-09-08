#include "lidar_undistortion/velodyne_undistorter_ros.hpp"
#include "lidar_undistortion/velodyne_container.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/node/detail/node.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace lidar_undistortion;
using namespace velodyne_pointcloud;

VelodyneUndistorterROS::VelodyneUndistorterROS(ros::NodeHandle& nh,
                                               ros::NodeHandle& private_nh)
  : fixed_frame_id_("odom"),
    lidar_frame_id_("velodyne"),
    tf_listener_(tf_buffer_)
{
  velodyne::RawDataConfig cfg;

  cfg.calibrationFile = data_.getCalibrationFilename(private_nh);
  cfg.max_range = 30;
  cfg.min_range = 0;
  cfg.model = "VLP16";
  cfg.view_direction = 0;
  cfg.view_width = 2*M_PI;

  data_.setup(cfg);

  rings_ = static_cast<uint8_t>(data_.numLasers());

  // set full 360 FoV and nominal range
  scan_sub_ = nh.subscribe("/velodyne_packets", 10, &VelodyneUndistorterROS::scanCallback, this);

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

void VelodyneUndistorterROS::scanCallback(const velodyne_msgs::VelodyneScan::ConstPtr &scan_msg){
  // // Create the corrected pointcloud ROS msg
  sensor_msgs::PointCloud2 pointcloud_corrected_msg;
  //
  // beams_ = scan_msg.packets.size();
  times_lut_.clear();
  auto pc = container_.getCloud();
  //
  // // clear input point cloud to handle this packet
  pc->points.clear();
  pc->width = 0;
  pc->height = 1;
  // process each packet provided by the driver

  int64_t time_start = scan_msg->packets.front().stamp.toNSec();
  int64_t time_end =  scan_msg->packets.back().stamp.toNSec();
  for (size_t next = 0; next < scan_msg->packets.size(); ++next) {
    // append all timings to the lut
    int32_t v = scan_msg->packets[next].stamp.toNSec() - time_start;
    std::vector<int32_t> t(data_.scansPerPacket(), v);
    times_lut_.insert(times_lut_.end(), t.begin(), t.end());

    // unpack the raw data and append to cloud
    data_.unpack(scan_msg->packets[next], container_, scan_msg->packets[next].stamp.toNSec());
  }

  if(!processCloud(pc, time_start)){
    return;
  }

  pcl::toROSMsg(*pc, pointcloud_corrected_msg);
  // Copy the pointcloud header correctly
  // NOTE: The header timestamp type in PCL pointclouds is narrower than in
  //       PointCloud2 msgs. We therefore copy this field directly from the
  //       losing timestamp accuracy.
  pointcloud_corrected_msg.header          = scan_msg->header;
  pointcloud_corrected_msg.header.stamp    = ros::Time().fromNSec(time_start);
  pointcloud_corrected_msg.header.frame_id = lidar_frame_id_;

  // publish the accumulated cloud message
  corrected_pointcloud_pub_.publish(pointcloud_corrected_msg);
}

void VelodyneUndistorterROS::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_msg){
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
        sensor_msgs::PointCloud2 out_msg;
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
