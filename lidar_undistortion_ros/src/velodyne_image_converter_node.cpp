#include "lidar_undistortion_ros/velodyne_image_converter.hpp"
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <velodyne_msgs/VelodyneScan.h>
#include <lidar_undistortion/ouster_point.hpp>

using namespace lidar_undistortion;
class VelodyneImageConverterNode {
public:
  using VelodyneCloud = VelodyneImageConverter::VelodyneCloud;
public:
  VelodyneImageConverterNode() = delete;
  VelodyneImageConverterNode(ros::NodeHandle& nh) : nh_(nh), img_transp_(nh_), img_cvt_(1824,16), scan_cvt_(nh_)
  {
    cloud_sub_ = nh_.subscribe("/velodyne_packets",10, &VelodyneImageConverterNode::velodyneCloudCallback, this);
    img_pub_ = img_transp_.advertise("velodyne_image", 1);
    range_in_.create(128, 1824, CV_64FC1);
    azimuth_in_.create(128, 1824, CV_64FC1);
    altitude_in_.create(128, 1824, CV_64FC1);
    intensity_in_.create(128, 1824, CV_64FC1);
    reflectivity_in_.create(128, 1824, CV_64FC1);
    ros::spin();
  }

  void velodyneCloudCallback(const velodyne_msgs::VelodyneScanConstPtr& msg)
  {
    // TODO some conversion from msg to cloud_in_
    scan_cvt_.scanToPointCloud(*msg, cloud_in_);
    img_cvt_.convert(cloud_in_, range_in_, altitude_in_, azimuth_in_, intensity_in_, reflectivity_in_);
    cv::Mat range_mono(range_in_.rows, range_in_.cols, CV_8UC1);
    img_cvt_.floatImageToMono(range_in_, range_mono);
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "mono8", range_mono).toImageMsg();
    img_pub_.publish(img_msg);
  }
private:
  ros::NodeHandle& nh_;
  image_transport::ImageTransport img_transp_;
  image_transport::Publisher img_pub_;
  VelodyneImageConverter img_cvt_;
  VelodyneScanConverter<PointOuster> scan_cvt_;
  ros::Subscriber cloud_sub_;
  VelodyneCloud cloud_in_;
  cv::Mat range_in_;
  cv::Mat azimuth_in_;
  cv::Mat altitude_in_;
  cv::Mat intensity_in_;
  cv::Mat reflectivity_in_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "velodyne_image_converter");
  ros::NodeHandle nh;
  VelodyneImageConverterNode node(nh);
  return 0;
}
