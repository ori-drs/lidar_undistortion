#pragma once

#include "lidar_undistortion/velodyne_point.hpp"
#include "lidar_undistortion/velodyne_container.hpp"

#include <velodyne_msgs/VelodyneScan.h>
#include <pcl/point_cloud.h>
#include "lidar_undistortion/velodyne_rawdata.hpp"

class VelodyneScanConverter {
public:
  VelodyneScanConverter(ros::NodeHandle& nh){
    velodyne::RawDataConfig cfg;

    cfg.calibrationFile = data_.getCalibrationFilename(nh);
    cfg.max_range = 100;
    cfg.min_range = 0;
    cfg.model = "VLP16";
    cfg.view_direction = 0;
    cfg.view_width = 2*M_PI;

    data_.setup(cfg);
  }


  template <class PointT>
  void scanToPointCloud(const velodyne_msgs::VelodyneScan& scan_msg,
                        std::vector<int32_t>& times_lut_,
                        pcl::PointCloud<PointT>& cloud) {
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

    int64_t time_start = scan_msg.packets.front().stamp.toNSec();
    for (size_t next = 0; next < scan_msg.packets.size(); ++next) {
      // append all timings to the lut
      int32_t v = scan_msg.packets[next].stamp.toNSec() - time_start;
      std::vector<int32_t> t(data_.scansPerPacket(), v);
      times_lut_.insert(times_lut_.end(), t.begin(), t.end());

      // unpack the raw data and append to cloud
      data_.unpack(scan_msg.packets[next], container_, scan_msg.packets[next].stamp.toNSec());
    }
    cloud = pc;
  }
private:
  velodyne::VelodyneContainer container_;              ///< input packet point cloud
  velodyne::RawData data_;
};
