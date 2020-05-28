#include "lidar_undistortion/velodyne_image_converter.hpp"
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <velodyne_msgs/VelodyneScan.h>

using namespace lidar_undistortion;
class VelodyneImageConverterNode {
public:
  using VelodyneCloud = VelodyneImageConverter::VelodyneCloud;
public:
  VelodyneImageConverterNode() = delete;
  VelodyneImageConverterNode(ros::NodeHandle& nh) : nh_(nh), img_transp_(nh_), img_cvt_(1024,64)
  {
    cloud_sub_ = nh_.subscribe("/velodyne_packets",10, &VelodyneImageConverterNode::velodyneCloudCallback, this);
    img_pub_ = img_transp_.advertise("velodyne_image", 1);
    range_in_.create(16, 1800, CV_64FC1);
    azimuth_in_.create(16, 1800, CV_64FC1);
    altitude_in_.create(16, 1800, CV_64FC1);
    ros::spin();
  }

  void velodyneCloudCallback(const velodyne_msgs::VelodyneScanConstPtr& msg)
  {
    // TODO some conversion from msg to cloud_in_
    img_cvt_.scanToPointCloud(*msg, cloud_in_);
    //img_cvt_.convert(cloud_in_, range_in_, altitude_in_, azimuth_in_);
    //cv::Mat range_mono(range_in_.rows, range_in_.cols, CV_8UC1);
    //img_cvt_.rangeImageToMono(range_in_, range_mono);
    //sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "mono8", range_mono).toImageMsg();
    //img_pub_.publish(img_msg);
  }
private:
  ros::NodeHandle& nh_;
  image_transport::ImageTransport img_transp_;
  image_transport::Publisher img_pub_;
  VelodyneImageConverter img_cvt_;
  ros::Subscriber cloud_sub_;
  VelodyneCloud cloud_in_;
  cv::Mat range_in_;
  cv::Mat azimuth_in_;
  cv::Mat altitude_in_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "ouster_image_converter");
  ros::NodeHandle nh;
  VelodyneImageConverterNode node(nh);
  return 0;
}
