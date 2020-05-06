#pragma once
#include "lidar_undistortion/lidar_undistorter.hpp"
#include <velodyne_pointcloud/datacontainerbase.h>
#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>

namespace lidar_undistortion {
class VelodyneUndistorter : public LidarUndistorter<velodyne_rawdata::VPoint> {

protected:
  velodyne_rawdata::RawData data_;

};
}
