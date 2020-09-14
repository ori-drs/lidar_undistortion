#include <ros/ros.h>
#include <ros/package.h>
#include "lidar_undistortion_ros/velodyne_undistorter_ros.hpp"

int main(int argc, char **argv) {
  // Register with ROS master
  ros::init(argc, argv, "velodyne_undistortion");

  // Create node handles
  ros::NodeHandle nh("~");

  // Launch the lidar undistorter
  lidar_undistortion::VelodyneUndistorterROS lidar_undistorter(nh);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
