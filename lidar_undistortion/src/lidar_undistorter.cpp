#include "lidar_undistortion/lidar_undistorter.h"


#include <eigen_utils/eigen_utils.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <iostream>


namespace lidar_undistortion {

bool OusterUndistorter::processCloud(const OusterCloud::Ptr &pointcloud,
                                    const uint64_t timestamp)
{
  // Assert that the pointcloud is not empty
  if (pointcloud->empty()){
    return false;
  }

  uint16_t timing_counter = 0;
  // save all the timings from the point cloud
  // this has redundancy because we copy the same value multiple times
  // but it is more generic because it does not assume the same timing occurs
  // from a column of the scanner
  for(auto it = pointcloud->points.begin(); it != pointcloud->points.end(); ++it){
    times_lut_[timing_counter++] = it->t;
  }

  return LidarUndistorter::processCloud(pointcloud, timestamp);
}



}  // namespace lidar_undistortion
