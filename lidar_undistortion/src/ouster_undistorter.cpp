#include "lidar_undistortion/ouster_undistorter.hpp"
#include "lidar_undistortion/lidar_image_converter.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <iostream>

namespace lidar_undistortion {

bool OusterUndistorter::processCloud(const OusterCloud::Ptr &pointcloud,
                                     const uint64_t timestamp,
                                     const uint64_t desired_timestamp) {
  // Assert that the pointcloud is not empty
  if (pointcloud->empty()) {
    DEBUG_PRINTLN("Point cloud empty!");
    return false;
  }
  size_t timing_counter = 0;
  // save all the timings from the point cloud
  // this has redundancy because we copy the same value multiple times
  // but it is more generic because it does not assume the same timing occurs
  // from a column of the scanner
  for (auto it = pointcloud->points.begin(); it != pointcloud->points.end();
       ++it) {
    times_lut_[timing_counter++] = it->t;
  }
  return LidarUndistorter::processCloud(pointcloud, timestamp,
                                        desired_timestamp);
}

bool OusterUndistorter::processCloud(const OusterCloud::Ptr &pointcloud,
                                     const uint64_t timestamp) {
  return processCloud(pointcloud, timestamp, timestamp);
}

}  // namespace lidar_undistortion
