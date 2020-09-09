#include <ros/ros.h>
#include <ros/package.h>
#include "lidar_undistortion_ros/velodyne_undistorter_ros.hpp"

int main(int argc, char **argv) {
  // Register with ROS master
  ros::init(argc, argv, "velodyne_undistortion");

  // Create node handles
  ros::NodeHandle nh;
  std::string velodyne_calibration_file;
  if (!nh.getParam("velodyne_calibration_file", velodyne_calibration_file)) {
    ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

    // have to use something: grab unit test version as a default
    std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
    velodyne_calibration_file = pkgPath + "/params/64e_utexas.yaml";
  }

  // Launch the lidar undistorter
  lidar_undistortion::VelodyneUndistorterROS lidar_undistorter(nh, velodyne_calibration_file);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
