#include "lidar_undistortion/lidar_undistorter.h"
#include <ouster_ros/point_os1.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <string>

namespace lidar_undistortion {
LidarUndistorter::LidarUndistorter(ros::NodeHandle nh,
                                   ros::NodeHandle nh_private)
    : fixed_frame_id_("odom"),
      lidar_frame_id_("os1_lidar"),
      tf_buffer_(ros::Duration(10)),
      tf_listener_(tf_buffer_) {
  // Subscribe to the undistorted pointcloud topic
  pointcloud_sub_ = nh.subscribe("pointcloud", 100,
                                 &LidarUndistorter::pointcloudCallback, this);

  // Advertise the corrected pointcloud topic
  corrected_pointcloud_pub_ = nh_private.advertise<sensor_msgs::PointCloud2>(
      "pointcloud_corrected", 100, false);

  pose_sub_ = nh.subscribe("/penguin/rovio/pose_with_covariance_stamped", 100, &LidarUndistorter::poseCallback, this);

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

void LidarUndistorter::pointcloudCallback(
    const sensor_msgs::PointCloud2 &pointcloud_msg) {
  // Convert the pointcloud to PCL
  pcl::PointCloud<ouster_ros::OS1::PointOS1> pointcloud;
  pcl::fromROSMsg(pointcloud_msg, pointcloud);

  // Assert that the pointcloud is not empty
  if (pointcloud.empty()) return;

  // Get the start and end times of the pointcloud
  ros::Time t_start = pointcloud_msg.header.stamp +
      ros::Duration(pointcloud.points.begin()->t * 1e-9);
  ros::Time t_end = pointcloud_msg.header.stamp +
      ros::Duration((--pointcloud.points.end())->t * 1e-9);

  Eigen::Isometry3d T_S_F_original;
  // Get the frame that the cloud should be expressed in
  if(!getInterpolatedPose(t_start.toNSec(), T_S_F_original)){
    ROS_WARN_STREAM("Couldn't get interpolated pose for time " << t_start.toSec());
  }

  // Compute the transform used to project the corrected pointcloud back into
  // lidar's scan frame, for more info see the current class' header

  // Correct the distortion on all points, using the LiDAR's true pose at
  // each point's timestamp
  uint32_t last_transform_update_t = 0;
  int n = 0;
  int same_t = 0;
  Eigen::Isometry3d T_S_original__S_corrected = Eigen::Isometry3d::Identity();
  for (ouster_ros::OS1::PointOS1 &point : pointcloud.points) {
    ROS_INFO_STREAM_COND(pointcloud_msg.header.seq==1552, "Point n " << n++ << "/" << pointcloud.points.size() << " of point cloud " << pointcloud_msg.header.seq);
    ROS_INFO_STREAM_COND(pointcloud_msg.header.seq==1552, "Point t " << point.t);
    ROS_INFO_STREAM_COND(pointcloud_msg.header.seq==1552, "Group same t " << same_t);

    // Check if the current point's timestamp differs from the previous one
    // If so, lookup the new corresponding transform
    if (point.t != last_transform_update_t) {
      ROS_INFO_STREAM_COND(pointcloud_msg.header.seq==1552,"point.t = " << point.t << " last_transform_update_t = " << last_transform_update_t);
      last_transform_update_t = point.t;
      ros::Time point_t =
          pointcloud_msg.header.stamp + ros::Duration(0, point.t);

      Eigen::Isometry3d T_F_S_correct;
      if(!getInterpolatedPose(point_t.toNSec(), T_F_S_correct)){
        ROS_WARN_STREAM("Couldn't get interpolated pose for time " << point_t.toSec());
        return;
      }
      T_S_original__S_corrected = T_S_F_original.inverse() * T_F_S_correct;
      same_t++;
    }

    // Correct the point's distortion, by transforming it into the fixed
    // frame based on the LiDAR sensor's current true pose, and then transform
    // it back into the lidar scan frame
    point = pcl::transformPoint(point, T_S_original__S_corrected.cast<float>());



    // Create the corrected pointcloud ROS msg
    sensor_msgs::PointCloud2 pointcloud_corrected_msg;
    pcl::toROSMsg(pointcloud, pointcloud_corrected_msg);

    // Copy the pointcloud header correctly
    // NOTE: The header timestamp type in PCL pointclouds is narrower than in
    //       PointCloud2 msgs. We therefore copy this field directly from the
    //       losing timestamp accuracy.
    pointcloud_corrected_msg.header = pointcloud_msg.header;

    // Publish the corrected pointcloud
    corrected_pointcloud_pub_.publish(pointcloud_corrected_msg);
  }
}


void LidarUndistorter::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_msg){
  Eigen::Isometry3d pose;
  tf2::fromMsg(pose_msg.pose.pose, pose);
  //@todo use move constructor for speedup
  odometry_history_[pose_msg.header.stamp.toNSec()] = pose * base_to_lidar_;
}

bool LidarUndistorter::getInterpolatedPose(const uint64_t &nsec,
                                            Eigen::Isometry3d& pose) const
{

    if(odometry_history_.empty() || nsec < odometry_history_.begin()->first || nsec > odometry_history_.rbegin()->first)
    {
        return false;
    }
    // this is the first element greater than utime
    PoseHistory::const_iterator it_low = odometry_history_.upper_bound(nsec);
    // if we have reached the bottom already, we return
    if(it_low == odometry_history_.end()){
        return false;
    }
    // now it_low contains the last element smaller than nsec
    --it_low;
    // if by chance we have the pose at that exact time, we return it
    if(it_low->first == nsec){
        pose = it_low->second;
        return true;
    }
    // at this point we have to interpolate, and we can't do it with less than
    // two items in history
    if(odometry_history_.size() < 2){
        return false;
    }
    // it_high is the last element with time greater than it_low
    PoseHistory::const_iterator it_high = std::prev(odometry_history_.upper_bound((std::next(it_low,1))->first),1);

    // alpha is 1 if the requested time coincides with it_low, 0 if equal to it_high
    double alpha = (double)(it_high->first - nsec) / (double)(it_high->first - it_low->first);

    Eigen::Isometry3d iso_low = it_low->second;
    Eigen::Isometry3d iso_high = it_high->second;

    pose.translation() = iso_low.translation() * alpha + iso_high.translation() * (1-alpha);
    // in slerp, the paramter t is used as the opposite of alpha
    // (1 - t) * p0 + t * p1

    // the "linear()" returns a reference to the rotation
    // matrix of the transform
    pose.linear() = Quaternion(iso_low.rotation()).slerp(1 - alpha, Quaternion(iso_high.rotation())).toRotationMatrix();
    return true;
}

bool LidarUndistorter::waitForTransform(const std::string &from_frame_id,
                                        const std::string &to_frame_id,
                                        const ros::Time &frame_timestamp,
                                        const double &sleep_between_retries__s,
                                        const double &timeout__s) {
  // Total time spent waiting for the updated pose
  ros::WallDuration t_waited(0.0);
  // Maximum time to wait before giving up
  ros::WallDuration t_max(timeout__s);
  // Timeout between each update attempt
  const ros::WallDuration t_sleep(sleep_between_retries__s);
  while (t_waited < t_max) {
    if (tf_buffer_.canTransform(to_frame_id, from_frame_id, frame_timestamp)) {
      return true;
    }
    t_sleep.sleep();
    t_waited += t_sleep;
  }
  ROS_WARN("Waited %.3fs, but still could not get the TF from %s to %s",
           t_waited.toSec(), from_frame_id.c_str(), to_frame_id.c_str());
  return false;
}
}  // namespace lidar_undistortion
