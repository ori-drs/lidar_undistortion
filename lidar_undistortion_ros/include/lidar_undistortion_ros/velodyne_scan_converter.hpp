#pragma once

#include <lidar_undistortion/velodyne_point.hpp>
#include <lidar_undistortion/velodyne_container.hpp>
#include <lidar_undistortion/velodyne_rawdata.hpp>

#include <velodyne_msgs/VelodyneScan.h>
#include <pcl/point_cloud.h>
#include <ros/node_handle.h>

template <class PointT>
class VelodyneScanConverter {
public:
  VelodyneScanConverter(ros::NodeHandle& nh){
    velodyne::RawDataConfig cfg;
    std::string velodyne_calib_file;

    if(!nh.getParam("velodyne_calibration_file", velodyne_calib_file)){
      ROS_FATAL_STREAM("Could not get calib file");
    }

    cfg.calibrationFile = velodyne_calib_file;
    cfg.max_range = 100;
    cfg.min_range = 0;
    cfg.model = "VLP16";
    cfg.view_direction = 0;
    cfg.view_width = 2*M_PI;

    data_.setup(cfg);
  }

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
      data_.unpack(reinterpret_cast<const velodyne::raw_packet_t*>(&scan_msg.packets[next].data[0]),
                   scan_msg.packets[next].stamp.toNSec(),
                   container_,
                   scan_msg.header.stamp.toNSec());
    }
    cloud = *pc;
  }
private:
  velodyne::VelodyneContainer<PointT> container_;              ///< input packet point cloud
  velodyne::RawData data_;
};
