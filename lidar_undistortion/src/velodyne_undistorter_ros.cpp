#include "lidar_undistortion/velodyne_undistorter_ros.hpp"
#include <velodyne_pointcloud/pointcloudXYZIR.h>

using namespace lidar_undistortion;
using namespace velodyne_pointcloud;

VelodyneUndistorterROS::VelodyneUndistorterROS(ros::NodeHandle node,
                                               ros::NodeHandle private_nh,
                                               const std::string &node_name)
  : calibration_(data_.setup(private_nh)),
    fixed_frame_id_("odom"),
    lidar_frame_id_("velodyne"),
    tf_listener_(tf_buffer_)
{
if(calibration_)
{
  ROS_DEBUG_STREAM("Calibration file loaded.");
  config_.num_lasers = static_cast<uint16_t>(calibration_->num_lasers);
}
else
{
  ROS_ERROR_STREAM("Could not load calibration file!");
}

// advertise output point cloud (before subscribing to input data)
point_cloud_pub_ =
  node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

scan_sub_ = node.subscribe("velodyne_packets", 10, &VelodyneUndistorterROS::scanCallback, this);

}

void VelodyneUndistorterROS::scanCallback(const velodyne_msgs::VelodyneScan &scan_msg){

}

