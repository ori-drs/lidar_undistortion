#pragma once

#include <lidar_undistortion/velodyne_point.hpp>
#include <lidar_undistortion/velodyne_container.hpp>
#include <lidar_undistortion/velodyne_rawdata.hpp>

#include <velodyne_msgs/VelodyneScan.h>
#include <pcl/point_cloud.h>
#include <ros/node_handle.h>
#include <ros/package.h>

namespace lidar_undistortion {

template <class PointT>
class VelodyneScanConverter {
public:
  VelodyneScanConverter() {
    // Setup with a default constructor for VLP16.
    ROS_WARN_STREAM("Setting up with default values for VLP-16 LIDAR");

    velodyne::RawDataConfig cfg;
    cfg.calibrationFile = ros::package::getPath("velodyne_pointcloud") + "/params/VLP16db.yaml";
    cfg.max_range = 100;
    cfg.min_range = 0;
    cfg.model = "VLP16";
    cfg.view_direction = 0;
    cfg.view_width = 2*M_PI;

    data_.setup(cfg);
  }

  VelodyneScanConverter(const velodyne::RawDataConfig& cfg) {
    data_.setup(cfg);
  }

  VelodyneScanConverter(ros::NodeHandle& nh){
    velodyne::RawDataConfig cfg;
    std::string velodyne_calibration_file;
    std::string velodyne_model = "VLP16";

    if(!nh.getParam("velodyne_model", velodyne_model)){
      ROS_WARN_STREAM("Could not get velodyne_model. Assuming VLP16");
    }

    if(!nh.getParam("velodyne_calibration_file", velodyne_calibration_file)){
      ROS_WARN_STREAM("Could not get calib file. Using default");
      // have to use something: grab unit test version as a default
      std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
      velodyne_calibration_file = pkgPath + "/params/VLP16db.yaml";
    }

    cfg.calibrationFile = velodyne_calibration_file;
    cfg.max_range = 100;
    cfg.min_range = 0;
    cfg.model = velodyne_model;
    cfg.view_direction = 0;
    cfg.view_width = 2*M_PI;

    data_.setup(cfg);
  }

  void scanToPointCloud(const velodyne_msgs::VelodyneScan& scan_msg,
                        pcl::PointCloud<PointT>& cloud) {
    //
    // beams_ = scan_msg.packets.size();
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

      // unpack the raw data and append to cloud
      data_.unpack(reinterpret_cast<const velodyne::raw_packet_t*>(&scan_msg.packets[next].data[0]),
                   scan_msg.packets[next].stamp.toNSec(),
                   container_,
                   time_start);
    }
    cloud = *pc;
  }
private:
  velodyne::VelodyneContainer<PointT> container_;              ///< input packet point cloud
  velodyne::RawData data_;
};
}
