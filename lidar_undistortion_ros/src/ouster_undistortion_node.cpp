#include <rclcpp/rclcpp.hpp>
#include "lidar_undistortion_ros/ouster_undistorter_ros.hpp"

int main(int argc, char **argv) {
  // Register with ROS master
  rclcpp::init(argc, argv)

  // Create node handles
  auto nh = rclcpp::Node::make_shared("lidar_undistortion");

  // Launch the lidar undistorter
  lidar_undistortion::OusterUndistorterROS lidar_undistorter(*nh);

  // Spin
  rclcpp::spin(nh);

  // Exit normally
  return 0;
}
