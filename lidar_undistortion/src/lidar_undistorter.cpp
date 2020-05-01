#include "lidar_undistortion/lidar_undistorter.h"

#include <pcl/common/transforms.h>
#include <eigen_utils/eigen_utils.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <iostream>
#include "lidar_undistortion/print_macros.hpp"

namespace lidar_undistortion {

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
  uint64_t t_end = timestamp + pointcloud->points.rbegin()->t;

  Eigen::Isometry3d T_S_F_original = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_S_F_end = Eigen::Isometry3d::Identity();

  // Get the frame that the cloud should be expressed in
  if(!odometry_history_.getInterpolatedPose(t_start, T_S_F_original))

  {
    DEBUG_PRINTLN("Couldn't get interpolated start pose for time " << t_start - time_offset
                    << "\n Starting time     : " << (odometry_history_.empty() ? std::string("none") : std::to_string(odometry_history_.startTime() - time_offset))
                    << "\n End time          : " << (odometry_history_.empty() ? std::string("none") : std::to_string(odometry_history_.endTime() - time_offset))
                    << "\n Pose history size : " << odometry_history_.size()
                    << "\n Cloud history size: " << cloud_history_.size());

    // if the cloud is not in the cloud buffer, we add it
    if(cloud_history_.find(timestamp) == cloud_history_.end()){
      cloud_history_[timestamp] = pointcloud;
      DEBUG_PRINTLN("Adding cloud to cloud history."
                << "\n Cloud history size: " << cloud_history_.size());
    }
    return false;
  } else {
    // add the just computed interpolated pose to the buffer of poses
    // in case we need it later
    odometry_history_.addPose(timestamp, T_S_F_original);
    DEBUG_PRINTLN("Adding interpolated pose to pose history"
                  << "\n Pose history size: " << odometry_history_.size());
  }

  // Check if the end pose is available and abort if not available
  if(!odometry_history_.getInterpolatedPose(t_end, T_S_F_end)){
    DEBUG_PRINTLN("Couldn't get interpolated end pose for time " << t_end - time_offset
                    << "\n Starting time     : " << (odometry_history_.empty() ? std::string("none") : std::to_string(odometry_history_.startTime()- time_offset))
                    << "\n End time          : " << (odometry_history_.empty() ? std::string("none") : std::to_string(odometry_history_.endTime() - time_offset))
                    << "\n Pose history size : " << odometry_history_.size()
                    << "\n Cloud history size: " << cloud_history_.size());

    // if the cloud is not in the cloud buffer, we add it
    if(cloud_history_.find(timestamp) == cloud_history_.end()){
      cloud_history_[timestamp] = pointcloud;
      DEBUG_PRINTLN("Adding cloud to cloud history."
                      << "\n Cloud history size: " << cloud_history_.size());
    }
    return false;
  } else {
    // add the just computed interpolated pose to the buffer of poses
    // in case we need it later
    odometry_history_.addPose(t_end, T_S_F_end);
    DEBUG_PRINTLN("Adding interpolated pose to pose history"
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
      if(!odometry_history_.getInterpolatedPose(point_t, T_F_S_correct)){
        DEBUG_PRINTLN("Couldn't get interpolated point pose for time " << point_t- time_offset
                        << "\n Starting time: " << (odometry_history_.empty() ? std::string("none") : std::to_string(odometry_history_.startTime() - time_offset))
                        << "\n End time     : " << (odometry_history_.empty() ? std::string("none") : std::to_string(odometry_history_.endTime() - time_offset)));
        // if the cloud is not in the cloud buffer, we add it
        if(cloud_history_.find(timestamp) == cloud_history_.end()){
          cloud_history_[timestamp] = pointcloud;
          DEBUG_PRINTLN("Adding cloud to cloud history."
                          << "\n Cloud history size: " << cloud_history_.size());
        }
        return false;
      } else {
        // add the just computed interpolated pose to the buffer of poses
        // in case we need it later
        odometry_history_.addPose(point_t, T_F_S_correct);
        DEBUG_PRINTLN("Adding interpolated pose to pose history"
                        << "\n Pose history size: " << odometry_history_.size());
      }
      T_S_original__S_corrected = T_S_F_original.inverse() * T_F_S_correct;

      same_t++;
    }

    // Correct the point's distortion, by transforming it into the fixed
    // frame based on the LiDAR sensor's current true pose, and then transform
    // it back into the lidar scan frame
    point = pcl::transformPoint(point, T_S_original__S_corrected.cast<float>());
    point.intensity = ((float)((double)point.t) * 1e-8);
  }

  if(timestamp == 1565309877706900736){
    pcl::PointCloud<ouster_ros::OS1::PointOS1> corrected_point_cloud_odom;
    DEBUG_PRINTLN("SAVING CLOUD");
    DEBUG_PRINTLN("Same T = " << same_t);
    DEBUG_PRINTLN("point cloud size: " << pointcloud->points.size());
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


void LidarUndistorter::addPose(uint64_t nsec, Eigen::Isometry3d &pose){
  //@todo use move constructor for speedup
  odometry_history_.addPose(nsec, pose * base_to_lidar_);
  DEBUG_PRINTLN("Pose history size: " << odometry_history_.size());

  // if there are point clouds in the buffer, we process them
  if(!cloud_history_.empty() && !odometry_history_.empty()){
    for(auto it = cloud_history_.lower_bound(odometry_history_.startTime()); it != cloud_history_.end(); ){
      DEBUG_PRINTLN("Processing cloud " << it->first - time_offset);
      if(!processCloud(it->second, it->first)){
        ++it;
      } else {
        DEBUG_PRINTLN("Processed. Cloud size is: " << cloud_history_.size());
        // the point cloud has been transformed successfully
        // we remove it from the buffer and return
        it = cloud_history_.erase(it);
      }
    }
  }

}






}  // namespace lidar_undistortion
