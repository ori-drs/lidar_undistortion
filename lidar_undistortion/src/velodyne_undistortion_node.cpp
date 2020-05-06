#include <ros/ros.h>
#include "lidar_undistortion/velodyne_undistorter_ros.hpp"

int main(int argc, char **argv) {
  // Register with ROS master
  ros::init(argc, argv, "velodyne_undistortion");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Launch the lidar undistorter
  lidar_undistortion::VelodyneUndistorterROS lidar_undistorter(nh, nh_private);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
