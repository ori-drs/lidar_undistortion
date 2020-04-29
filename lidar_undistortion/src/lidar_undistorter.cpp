#include "lidar_undistortion/lidar_undistorter.h"

#include <pcl/common/transforms.h>

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

bool LidarUndistorter::processCloud(const OusterCloud::Ptr &pointcloud,
                                    const uint64_t timestamp)
{
  // Assert that the pointcloud is not empty
  if (pointcloud->empty()){
    return false;
  }

  // Get the start and end times of the pointcloud
  // t_start should be the same as timestamp
  uint64_t t_start = timestamp + pointcloud->points.begin()->t;

  Eigen::Isometry3d T_S_F_original;
  // Get the frame that the cloud should be expressed in
  if(!getInterpolatedPose(t_start, T_S_F_original)){
    ROS_WARN_STREAM("Couldn't get interpolated start pose for time " << t_start - time_offset
                    << "\n Starting time: " << odometry_history_.begin()->first - time_offset
                    << "\n End time     : " << odometry_history_.rbegin()->first- time_offset);

    // if the cloud is not in the cloud buffer, we add it
    if(cloud_history_.find(timestamp) == cloud_history_.end()){

      cloud_history_[timestamp] = pointcloud;
      ROS_WARN_STREAM("Adding cloud to cloud history."
                      << "\n Cloud history size: " << cloud_history_.size());
    }
    return false;
  } else {
    // add the just computed interpolated pose to the buffer of poses
    // in case we need it later

    odometry_history_[timestamp] = T_S_F_original;
    ROS_INFO_STREAM("Adding interpolated pose to pose history"
                    << "\n Pose history size: " << odometry_history_.size());
  }

  // Compute the transform used to project the corrected pointcloud back into
  // lidar's scan frame, for more info see the current class' header

  // Correct the distortion on all points, using the LiDAR's true pose at
  // each point's timestamp
  uint32_t last_transform_update_t = 0;
  int same_t = 0;
  Eigen::Isometry3d T_S_original__S_corrected = Eigen::Isometry3d::Identity();
  for (ouster_ros::OS1::PointOS1 &point : pointcloud->points) {
    // Check if the current point's timestamp differs from the previous one
    // If so, lookup the new corresponding transform
    if (point.t != last_transform_update_t) {
      last_transform_update_t = point.t;
      uint64_t point_t = timestamp + point.t;

      Eigen::Isometry3d T_F_S_correct;
      if(!getInterpolatedPose(point_t, T_F_S_correct)){
        ROS_WARN_STREAM("Couldn't get interpolated point pose for time " << point_t- time_offset
                        << "\n Starting time: " << odometry_history_.begin()->first- time_offset
                        << "\n End time     : " << odometry_history_.rbegin()->first- time_offset);
        // if the cloud is not in the cloud buffer, we add it
        if(cloud_history_.find(timestamp) == cloud_history_.end()){
          cloud_history_[timestamp] = pointcloud;
          ROS_WARN_STREAM("Adding cloud to cloud history."
                          << "\n Cloud history size: " << cloud_history_.size());
        }
        return false;
      }
      T_S_original__S_corrected = T_S_F_original.inverse() * T_F_S_correct;
      same_t++;
    }

    // Correct the point's distortion, by transforming it into the fixed
    // frame based on the LiDAR sensor's current true pose, and then transform
    // it back into the lidar scan frame
    point = pcl::transformPoint(point, T_S_original__S_corrected.cast<float>());
  }

  if(timestamp == 1565309877706900736){
    pcl::PointCloud<ouster_ros::OS1::PointOS1> corrected_point_cloud_odom;

    std::stringstream ss;
    std::string path = "/home/mcamurri/Datasets/lidar_undistortion_moog/clouds/";
    uint64_t sec = timestamp / 1000000000;
    uint64_t nsec = timestamp - sec * 1000000000;
    ss << "cloud_" << sec << "_" << std::setw(9) << std::setfill('0')
       << nsec;

    pcl::transformPointCloud(*pointcloud, corrected_point_cloud_odom, T_S_F_original.cast<float>());
    pcl::io::savePCDFile(path + ss.str() + "_fixed_new.pcd", corrected_point_cloud_odom);
  }

  return true;
}

void LidarUndistorter::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg) {
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

  // DEBUGGING


  // Publish the corrected pointcloud
  corrected_pointcloud_pub_.publish(pointcloud_corrected_msg);

}


void LidarUndistorter::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_msg){
  ROS_INFO_STREAM("Gotten pose " << pose_msg.header.stamp.toNSec()- time_offset);
  Eigen::Isometry3d pose;
  tf2::fromMsg(pose_msg.pose.pose, pose);
  //@todo use move constructor for speedup
  odometry_history_[pose_msg.header.stamp.toNSec()] = pose * base_to_lidar_;
  ROS_INFO_STREAM("Pose history size: " << odometry_history_.size());

  // if there are point clouds in the buffer, we process them
  if(!cloud_history_.empty()){
    for( auto it = cloud_history_.begin(); it != cloud_history_.end(); ){
      ROS_INFO_STREAM("Processing cloud " << it->first - time_offset);
      if(!processCloud(it->second, it->first)){
        break;
      } else {
      ROS_INFO_STREAM("Processed. Cloud size is: " << cloud_history_.size());
      // the point cloud has been transformed successfully
      // we remove it from the buffer and return
      it = cloud_history_.erase(it);
      }
    }
  }
  // clean up history longer than 1 s
  ROS_INFO_STREAM("Check if history has to be cleaned...");
  const auto& old_it = odometry_history_.lower_bound(odometry_history_.rbegin()->first - 1000000000);

  if(old_it != odometry_history_.begin()){
    ROS_INFO_STREAM("Erasing history prior to " << odometry_history_.rbegin()->first - 1000000000 - time_offset);
    odometry_history_.erase(odometry_history_.begin(), old_it);
  } else {
    ROS_INFO_STREAM("No cleaning necessary.");
  }

}

bool LidarUndistorter::getInterpolatedPose(const uint64_t &nsec,
                                           Eigen::Isometry3d& pose) const
{

  if(odometry_history_.empty() || nsec < odometry_history_.begin()->first || nsec > odometry_history_.rbegin()->first)
  {
    ROS_INFO_STREAM("[ getInterpolatedPose ]: empty history or requested time out of bounds. "
                    " Pose history size: " << odometry_history_.size());
    return false;
  }
  // this is the first element greater than utime
  PoseHistory::const_iterator it_low = odometry_history_.upper_bound(nsec);
  // if we have reached the bottom already, we return
  if(it_low == odometry_history_.end()){
    ROS_INFO_STREAM("[ getInterpolatedPose ]: reached bottom. "
                    "\n Pose history size: " << odometry_history_.size());
    return false;
  }
  // now it_low contains the last element smaller than nsec
  --it_low;
  // if by chance we have the pose at that exact time, we return it
  if(it_low->first == nsec){
    pose = it_low->second;
    ROS_INFO_STREAM("[ getInterpolatedPose ]: " << it_low->first << " == " << nsec <<
                    " Returning pose. \n Pose history size: " << odometry_history_.size());

    return true;
  }
  // at this point we have to interpolate, and we can't do it with less than
  // two items in history
  if(odometry_history_.size() < 2){
    ROS_INFO_STREAM("[ getInterpolatedPose ]: size less than 2. "
                    "\n history size: " << odometry_history_.size());

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
