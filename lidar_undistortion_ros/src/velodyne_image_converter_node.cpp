#include "lidar_undistortion_ros/velodyne_image_converter.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>
#include <lidar_undistortion/ouster_point.hpp>

using namespace lidar_undistortion;
class VelodyneImageConverterNode {
public:
  using VelodyneCloud = VelodyneImageConverter::VelodyneCloud;
public:
  VelodyneImageConverterNode() = delete;
  VelodyneImageConverterNode(rclcpp::Node& nh) : nh_(nh), img_cvt_(1824,16), scan_cvt_(nh_)
  {
    cloud_sub_ = nh_.create_subscription<velodyne_msgs::msg::VelodyneScan>(
        "velodyne_packets", 10, std::bind(&VelodyneImageConverterNode::velodyneCloudCallback, this, std::placeholders::_1));
    img_pub_ = nh_.create_publisher<sensor_msgs::msg::Image>("velodyne_image", 1);
    range_in_.create(128, 1824, CV_64FC1);
    azimuth_in_.create(128, 1824, CV_64FC1);
    altitude_in_.create(128, 1824, CV_64FC1);
    intensity_in_.create(128, 1824, CV_64FC1);
    reflectivity_in_.create(128, 1824, CV_64FC1);
    // rclcpp::spin(nh);
  }

  void velodyneCloudCallback(const velodyne_msgs::msg::VelodyneScan::ConstPtr& msg)
  {
    // TODO some conversion from msg to cloud_in_
    scan_cvt_.scanToPointCloud(*msg, cloud_in_);
    img_cvt_.convert(cloud_in_, range_in_, altitude_in_, azimuth_in_, intensity_in_, reflectivity_in_);
    cv::Mat range_mono(range_in_.rows, range_in_.cols, CV_8UC1);
    img_cvt_.floatImageToMono(range_in_, range_mono);
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(msg->header, "mono8", range_mono).toImageMsg();
    img_pub_->publish(*img_msg.get());
  }
private:
  rclcpp::Node& nh_;
  // image_transport::ImageTransport img_transp_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  VelodyneImageConverter img_cvt_;
  VelodyneScanConverter<PointOuster> scan_cvt_;
  rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr cloud_sub_;
  VelodyneCloud cloud_in_;
  cv::Mat range_in_;
  cv::Mat azimuth_in_;
  cv::Mat altitude_in_;
  cv::Mat intensity_in_;
  cv::Mat reflectivity_in_;

};

int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("velodyne_image_converter");
  VelodyneImageConverterNode lidar_undistorter(*nh);
  rclcpp::spin(nh);
  return 0;
}
