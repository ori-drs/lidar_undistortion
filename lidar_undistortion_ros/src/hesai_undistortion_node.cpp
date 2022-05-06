#include <ros/ros.h>
#include "lidar_undistortion_ros/hesai_undistorter_ros.hpp"

int main(int argc, char **argv) {
  // Register with ROS master
  ros::init(argc, argv, "hesai_undistortion");

  // Create node handles
  ros::NodeHandle nh("~");

  // Launch the lidar undistorter
  lidar_undistortion::HesaiUndistorterROS lidar_undistorter(nh);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
