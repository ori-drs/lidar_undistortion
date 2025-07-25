#include "lidar_undistortion_ros/ouster_undistorter_ros.hpp"
#include <lidar_undistortion/ouster_metadata_utils.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <ouster_ros/OSConfigSrv.h>
// #include <ros/package.h>


typedef std::chrono::high_resolution_clock clock_;
typedef std::chrono::duration<double, std::ratio<1> > second_;

using namespace lidar_undistortion;
using namespace boost::filesystem;
using namespace ouster::sensor;

OusterUndistorterROS::OusterUndistorterROS(rclcpp::Node& nh)
  : OusterUndistorter(2e9),
    fixed_frame_id_("odom"),
    lidar_frame_id_("os_sensor"),
    img_transp_(nh),
    corrected_range_pub_(img_transp_.advertise("corrected_range",1)),
    tf_listener_(tf_buffer_)
{
  nh.get_parameter("point_cloud_input_topic", point_cloud_input_topic_);
  nh.get_parameter("pose_topic", pose_topic_);
  nh.get_parameter("point_cloud_output_topic", point_cloud_output_topic_);

  // Subscribe to the undistorted pointcloud topic
  pointcloud_sub_ = nh.subscribe(point_cloud_input_topic_, 100,
                                 &OusterUndistorterROS::pointcloudCallback, this);

  // Advertise the corrected pointcloud topic
  corrected_pointcloud_pub_ = nh.create_publisher<sensor_msgs::msg::PointCloud2>(
        point_cloud_output_topic_, 100);

  pose_sub_ = nh.subscribe(pose_topic_, 100, &OusterUndistorterROS::poseCallback, this);

  // Read the odom and lidar frame names from ROS params
  if(!nh.param("fixed_frame_id", fixed_frame_id_, fixed_frame_id_)){
    RCLCPP_WARN_STREAM(nh.get_logger(), "Could not read param \"fixed_frame_id\". "
                    << "Setting to default: " << fixed_frame_id_);
  }
  if(!nh.param("lidar_frame_id", lidar_frame_id_, lidar_frame_id_)){
    RCLCPP_WARN_STREAM(nh.get_logger(), "Could not read param \"lidar_frame_id\". "
                    << "Setting to default: " << lidar_frame_id_);
  }
  if(!nh.param("base_frame_id", base_frame_id_, base_frame_id_)){
    RCLCPP_WARN_STREAM(nh.get_logger(), "Could not read param \"base_frame_id\". "
                    << "Setting to default: " << base_frame_id_);
  }

  // 1. try to get the info from the active client from the driver
  ouster_ros::OSConfigSrv cfg_srv{};

  sensor_info info;

  ros::ServiceClient client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
  if (client.call(cfg_srv)) {
    info  = parse_metadata(cfg_srv.response.metadata);
  } else {
    RCLCPP_WARN_STREAM(nh.get_logger(), "Could not get the ROS client to get sensor info. "
                    << "Attempting to read file name from param server.");
    std::string json_cfg_file;
    // 2. if server unavailable, load the file from parameter server and parse it
    if(nh.get_parameter("ouster_config_file", json_cfg_file) &&
       is_regular_file(path(json_cfg_file)))
    {
      std::string metadata_string = read_metadata(json_cfg_file);
      // std::cerr << metadata_string << std::endl;
      info  = parse_metadata(metadata_string);
    } else {
      // 3. if file is not available, fill in with default values from OS1-64 Gen1
      RCLCPP_WARN_STREAM(nh.get_logger(), "Could not read param \"config_file\" from param server. "
                      << "Setting to default values for OS1-64 Gen1");
    }
  }
  // fill in all the values that hasn't been filled in by previous attempts
  populate_metadata_defaults(info, MODE_1024x10);

  OusterConfig os_cfg(info);

  beams_ = os_cfg.W;
  rings_ = os_cfg.H;
  cvt_ = std::make_unique<OusterImageConverter>(os_cfg);

  setRingsAndBeams(rings_, beams_);

  ranges_.create(rings_,beams_, CV_64FC1);
  altitudes_.create(rings_,beams_, CV_64FC1);
  azimuths_.create(rings_,beams_, CV_64FC1);
  intensities_.create(rings_,beams_, CV_64FC1);
  reflectivities_.create(rings_,beams_, CV_64FC1);
  ranges_viz_.create(rings_, beams_, CV_8UC1);

  // retrieve the transform from base to lidar frame
  while(nh.ok()){
    try{
      geometry_msgs::msg::TransformStamped temp_transform;
      temp_transform = tf_buffer_.lookupTransform(base_frame_id_, lidar_frame_id_,
                                                  ros::Time(0));

      base_to_lidar_ = tf2::transformToEigen(temp_transform);
      break;
    }
    catch (const tf2::TransformException& ex){
      rclcpp_ERROR(nh.get_logger(), "%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  RCLCPP_INFO(nh.get_logger(), "OusterUndistorterROS ready.");
}

void OusterUndistorterROS::pointcloudCallback(const sensor_msgs::msg::PointCloud2& pointcloud_msg) {
  // Convert the pointcloud to PCL
  OusterCloud::Ptr pointcloud = pcl::make_shared<OusterCloud>();
  pcl::fromROSMsg(pointcloud_msg, *pointcloud);

  // std::chrono::time_point<clock_> beg_ = clock_::now();
  if(!processCloud(pointcloud, pointcloud_msg.header.stamp.nanosec)){
    ROS_DEBUG_STREAM("processCloud for ns=" << pointcloud_msg.header.stamp.nanosec << " returned false, ignoring point cloud.");
    return;
  }
  // std::cout << "Time for reprocessCloudBuffer: " << std::chrono::duration_cast<second_> (clock_::now() - beg_).count() << std::endl;

  // Create the corrected pointcloud ROS msg
  sensor_msgs::msg::PointCloud2 pointcloud_corrected_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_corrected_msg);
  // Copy the pointcloud header correctly
  // NOTE: The header timestamp type in PCL pointclouds is narrower than in
  //       PointCloud2 msgs. We therefore copy this field directly from the
  //       losing timestamp accuracy.
  pointcloud_corrected_msg.header = pointcloud_msg.header;
  pointcloud_corrected_msg.header.frame_id = lidar_frame_id_;

  // Publish the corrected pointcloud
  corrected_pointcloud_pub_->publish(pointcloud_corrected_msg);
}

void OusterUndistorterROS::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &pose_msg){
  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  tf2::fromMsg(pose_msg.pose.pose, pose);
  addPose(pose_msg.header.stamp.nanosec, pose);
  reprocessCloudBuffer();
}

void OusterUndistorterROS::reprocessCloudBuffer(){
  // if there are point clouds in the buffer, we process them
  if(!cloud_history_.empty() && !odometry_history_.empty()){
    for(auto it = cloud_history_.begin(); it != cloud_history_.end(); ){
      DEBUG_PRINTLN("Processing cloud " << it->first);
      if(!processCloud(it->second, it->first)){
        if (it->first < odometry_history_.startTime()) {
          it = cloud_history_.erase(it);
        } else {
          ++it;
        }
      } else {
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*it->second, out_msg);
        out_msg.header.stamp = ros::Time().fromNSec(it->first);
        out_msg.header.frame_id = lidar_frame_id_;
        corrected_pointcloud_pub_->publish(out_msg);
        DEBUG_PRINTLN("Processed. Cloud size is: " << cloud_history_.size());

        cvt_->convert(*it->second, ranges_, altitudes_, azimuths_, intensities_, reflectivities_);
        cvt_->floatImageToMono(ranges_, ranges_viz_);

        range_img_msg_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", ranges_viz_).toImageMsg();

        range_img_msg_->header.stamp.fromNSec(it->first);
        corrected_range_pub_->publish(range_img_msg_);

        // the point cloud has been transformed successfully
        // we remove it from the buffer and return
        it = cloud_history_.erase(it);
      }
    }
  }
}
