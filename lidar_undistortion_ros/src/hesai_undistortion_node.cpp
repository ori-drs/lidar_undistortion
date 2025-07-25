#include <rclcpp/rclcpp.hpp>
#include "lidar_undistortion_ros/hesai_undistorter_ros.hpp"

int main(int argc, char **argv) {
  // Register with ROS master
  rclcpp::init(argc, argv);

  // Create node handles
  auto nh = rclcpp::Node::make_shared("hesai_undistortion");

  // Launch the lidar undistorter
  lidar_undistortion::HesaiUndistorterROS lidar_undistorter(*nh);

  // Spin
  rclcpp::spin(nh);

  // Exit normally
  return 0;
}
