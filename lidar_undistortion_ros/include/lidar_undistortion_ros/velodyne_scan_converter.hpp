#pragma once

#include <lidar_undistortion/velodyne_point.hpp>
#include <lidar_undistortion/velodyne_container.hpp>
#include <lidar_undistortion/velodyne_rawdata.hpp>

#include <velodyne_msgs/msg/velodyne_scan.hpp>
#include <pcl/point_cloud.h>
#include <rclcpp/node.hpp>
// #include <ros/package.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
namespace lidar_undistortion {

template <class PointT>
class VelodyneScanConverter {
public:
  VelodyneScanConverter(bool print = true) {
    // Setup with a default constructor for VLP16.
    // RCLCPP_WARN_STREAM("Setting up with default values for VLP-16 LIDAR");

    velodyne::RawDataConfig cfg;

    std::string cfg_file; // = ros::package::getPath("lidar_undistortion") + "/config/VLP16db_example.yaml";
    boost::filesystem::path cfg_path(cfg_file);

    if(!boost::filesystem::is_regular_file(cfg_path)){
      // RCLCPP_FATAL_STREAM("Could not load file " << cfg_file << " for Velodyne calibration!");
      return;
    }

    cfg.calibrationFile = boost::filesystem::canonical(cfg_path).string();
    cfg.max_range = 100;
    cfg.min_range = 0;
    cfg.model = "VLP16";
    cfg.view_direction = 0;
    cfg.view_width = 2*M_PI;

    if(!data_.setup(cfg, print)){
      throw std::runtime_error("Could not setup the velodyne scan converter. Invalid Velodyne parameters.");
    }
  }

  VelodyneScanConverter(const velodyne::RawDataConfig& cfg) {
    if(!data_.setup(cfg)){
      throw std::runtime_error("Could not setup the velodyne scan converter. Invalid Velodyne parameters.");
    }
  }

  VelodyneScanConverter(rclcpp::Node& nh) : VelodyneScanConverter(false) {
    // get the velodyne config already modified by the default constructor
    velodyne::RawDataConfig cfg = data_.config();

    std::string velodyne_calibration_file;
    std::string velodyne_model = "VLP16";

    // update the relevant fields of the velodyne config from ROS
    if(!nh.getParam("velodyne_model", velodyne_model)){
      RCLCPP_WARN_STREAM(nh.get_logger(), "Could not get velodyne_model. Assuming VLP16");
    } else {
      cfg.model = velodyne_model;
    }

    if(!nh.getParam("velodyne_calibration_file", velodyne_calibration_file)){
      RCLCPP_WARN_STREAM(nh.get_logger(), "Could not get calib file. Using default");
    } else {
      cfg.calibrationFile = velodyne_calibration_file;
    }

    // setup the updated velodyne config
    if(!data_.setup(cfg)){
      throw std::runtime_error("Could not setup the velodyne scan converter. Invalid Velodyne parameters.");
    }
  }

  void scanToPointCloud(const velodyne_msgs::msg::VelodyneScan& scan_msg,
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
