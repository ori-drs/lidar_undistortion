#include "lidar_undistortion/ouster_image_converter.hpp"
#include <lidar_undistortion/RangeImage.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

using namespace lidar_undistortion;
class OusterImageConverterNode {
public:
  using OusterCloud = OusterImageConverter::OusterCloud;
public:
  OusterImageConverterNode() = delete;
  OusterImageConverterNode(ros::NodeHandle& nh) : nh_(nh), img_transp_(nh_), img_cvt_(1024,64)
  {
    cloud_sub_ = nh_.subscribe("/os1_cloud_node/points",10, &OusterImageConverterNode::ousterCloudCallback, this);
    range_img_pub_ = img_transp_.advertise("ouster_range_image", 1);
    intensity_img_pub_ = img_transp_.advertise("ouster_intensity_image",1);
    reflectivity_img_pub_ = img_transp_.advertise("ouster_reflectivity_image",1);
    range_data_pub_ = nh_.advertise<lidar_undistortion::RangeImage>("ouster_range_data",1);
    range_in_.create(64, 1024, CV_64FC1);
    azimuth_in_.create(64, 1024, CV_64FC1);
    altitude_in_.create(64, 1024, CV_64FC1);
    intensity_in_.create(64, 1024, CV_64FC1);
    reflectivity_in_.create(64, 1024, CV_64FC1);
    range_msg_.height = 64;
    range_msg_.width = 1024;
    range_msg_.ranges.resize(range_msg_.height * range_msg_.width);
    range_msg_.azimuths.resize(range_msg_.height * range_msg_.width);
    range_msg_.altitude.resize(range_msg_.height * range_msg_.width);

    ros::spin();
  }

  void ousterCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    pcl::fromROSMsg(*msg, cloud_in_);

    img_cvt_.convert(cloud_in_,
                     range_in_,
                     altitude_in_,
                     azimuth_in_,
                     intensity_in_,
                     reflectivity_in_);
    // TODO check if there are memory tricks to avoid this deep copy
    // with a for loop
    range_msg_.header = msg->header;

    for(size_t i = 0; i < range_msg_.height; i++){
      for(size_t j = 0; j < range_msg_.width; j++){
        range_msg_.ranges[i*range_msg_.width + j] = range_in_.at<double>(i,j);
        range_msg_.azimuths[i*range_msg_.width + j] = azimuth_in_.at<double>(i,j);
        range_msg_.altitude[i*range_msg_.width + j] = altitude_in_.at<double>(i,j);
      }
    }
    range_data_pub_.publish(range_msg_);

    cv::Mat range_mono(range_in_.rows, range_in_.cols, CV_8UC1);
    img_cvt_.floatImageToMono(range_in_, range_mono);
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "mono8", range_mono).toImageMsg();
    range_img_pub_.publish(img_msg);
    cv::Mat intensity_mono(intensity_in_.rows, intensity_in_.cols, CV_8UC1);
    img_cvt_.floatImageToMono(intensity_in_, intensity_mono, 4096);
    img_msg = cv_bridge::CvImage(msg->header, "mono8", intensity_mono).toImageMsg();
    intensity_img_pub_.publish(img_msg);
    cv::Mat reflectivity_mono(reflectivity_in_.rows, reflectivity_in_.cols, CV_8UC1);
    img_cvt_.floatImageToMono(reflectivity_in_, reflectivity_mono, 9000);
    img_msg = cv_bridge::CvImage(msg->header, "mono8", reflectivity_mono).toImageMsg();
    reflectivity_img_pub_.publish(img_msg);
  }
private:
  ros::NodeHandle& nh_;
  image_transport::ImageTransport img_transp_;
  image_transport::Publisher range_img_pub_;
  image_transport::Publisher intensity_img_pub_;
  image_transport::Publisher reflectivity_img_pub_;
  ros::Publisher range_data_pub_;
  OusterImageConverter img_cvt_;
  RangeImage range_msg_;

  OusterCloud cloud_in_;
  ros::Subscriber cloud_sub_;


  cv::Mat range_in_;
  cv::Mat azimuth_in_;
  cv::Mat altitude_in_;
  cv::Mat intensity_in_;
  cv::Mat reflectivity_in_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "ouster_image_converter");
  ros::NodeHandle nh;
  OusterImageConverterNode node(nh);
  return 0;
}
