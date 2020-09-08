#include "lidar_undistortion/ouster_undistorter_ros.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

typedef std::chrono::high_resolution_clock clock_;
typedef std::chrono::duration<double, std::ratio<1> > second_;

using namespace lidar_undistortion;

OusterUndistorterROS::OusterUndistorterROS(ros::NodeHandle nh,
                                         ros::NodeHandle nh_private)
  : fixed_frame_id_("odom"),
    lidar_frame_id_("os1_lidar"),
    tf_listener_(tf_buffer_),
    OusterUndistorter(2e9),
    img_transp_(nh_private),
    corrected_range_pub_(img_transp_.advertise("corrected_range",1))
{
  // Subscribe to the undistorted pointcloud topic
  pointcloud_sub_ = nh.subscribe("pointcloud", 100,
                                 &OusterUndistorterROS::pointcloudCallback, this);

  // Advertise the corrected pointcloud topic
  corrected_pointcloud_pub_ = nh_private.advertise<sensor_msgs::PointCloud2>(
        "pointcloud_corrected", 100, false);

  nh_private.param("pose_topic", pose_topic_, pose_topic_);

  pose_sub_ = nh.subscribe(pose_topic_, 100, &OusterUndistorterROS::poseCallback, this);

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

void OusterUndistorterROS::pointcloudCallback(const sensor_msgs::PointCloud2& pointcloud_msg) {
  // Convert the pointcloud to PCL
  OusterCloud::Ptr pointcloud = boost::make_shared<OusterCloud>();
  pcl::fromROSMsg(pointcloud_msg, *pointcloud);

  // std::chrono::time_point<clock_> beg_ = clock_::now();
  if(!processCloud(pointcloud, pointcloud_msg.header.stamp.toNSec())){
    return;
  };
  // std::cout << "Time for reprocessCloudBuffer: " << std::chrono::duration_cast<second_> (clock_::now() - beg_).count() << std::endl;

  // Create the corrected pointcloud ROS msg
  sensor_msgs::PointCloud2 pointcloud_corrected_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_corrected_msg);
  // Copy the pointcloud header correctly
  // NOTE: The header timestamp type in PCL pointclouds is narrower than in
  //       PointCloud2 msgs. We therefore copy this field directly from the
  //       losing timestamp accuracy.
  pointcloud_corrected_msg.header = pointcloud_msg.header;
  pointcloud_corrected_msg.header.frame_id = lidar_frame_id_;

  // Publish the corrected pointcloud
  corrected_pointcloud_pub_.publish(pointcloud_corrected_msg);
}

void OusterUndistorterROS::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_msg){
  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  tf2::fromMsg(pose_msg.pose.pose, pose);
  addPose(pose_msg.header.stamp.toNSec(), pose);
  reprocessCloudBuffer();
}

void OusterUndistorterROS::reprocessCloudBuffer(){
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
        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(*it->second, out_msg);
        out_msg.header.stamp = ros::Time().fromNSec(it->first);
        out_msg.header.frame_id = lidar_frame_id_;
        corrected_pointcloud_pub_.publish(out_msg);
        DEBUG_PRINTLN("Processed. Cloud size is: " << cloud_history_.size());

        cvt_->convert(*it->second, ranges_, altitudes_, azimuths_, intensities_, reflectivities_);
        cvt_->floatImageToMono(ranges_, ranges_viz_);

        range_img_msg_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", ranges_viz_).toImageMsg();

        range_img_msg_->header.stamp.fromNSec(it->first);
        corrected_range_pub_.publish(range_img_msg_);

        // the point cloud has been transformed successfully
        // we remove it from the buffer and return
        it = cloud_history_.erase(it);
      }
    }
  }
}
