#ifndef LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_
#define LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Eigen>
#include <string>
#include <pcl/point_cloud.h>
#include <ouster_ros/point_os1.h>

namespace lidar_undistortion {
class LidarUndistorter {
public:
  const uint64_t time_offset = 0;//1565309854000000000;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PoseHistory = std::map<uint64_t, Eigen::Isometry3d, std::less<uint64_t>,
  Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Isometry3d> > >;
  using OusterCloud = pcl::PointCloud<ouster_ros::OS1::PointOS1>;

  using CloudHistory = std::map<uint64_t, OusterCloud::Ptr>;
  using PosePair = std::pair<uint64_t, Eigen::Isometry3d>;

  using Vector3d = Eigen::Vector3d;
  using Quaternion = Eigen::Quaterniond;


 public:
  LidarUndistorter(ros::NodeHandle nh, ros::NodeHandle nh_private);

  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg);
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);

 private:
  // TF frame name of the lidar scan frame
  std::string lidar_frame_id_;

  // TF frame name of a frame that can be considered fixed
  // NOTE: When correcting the pointcloud distortion, each point is first
  //       transformed into a fixed frame (F), using the lidar's true pose
  //       at the time that the point was recorded (S_correct).
  //       The point is then transformed back into the scan frame (S_original)
  //       matching the pointcloud message's frame_id and timestamp.
  std::string fixed_frame_id_;
  std::string base_frame_id_ = "imu";

  // ROS subscriber and publisher for the (un)corrected pointclouds
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber pose_sub_;

  ros::Publisher corrected_pointcloud_pub_;

  // Members used to lookup TF transforms
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Eigen::Isometry3d base_to_lidar_ = Eigen::Isometry3d::Identity();

  // Method that waits for a transform to become available, while doing less
  // agressive polling that ROS's standard tf2_ros::Buffer::canTransform(...)
  bool waitForTransform(const std::string &from_frame_id,
                        const std::string &to_frame_id,
                        const ros::Time &frame_timestamp,
                        const double &sleep_between_retries__s,
                        const double &timeout__s);

  // Inline method to convert ROS transform msgs to Eigen Affine transforms
  // NOTE: This is a copy of the tf::transformMsgToEigen() method from
  //       eigen_conversions/eigen_msg.h that has been modified such that float
  //       precision can be used (e.g. for Eigen::Affine3f transforms)
  template <typename T>
  void transformMsgToEigen(const geometry_msgs::Transform &transform_msg,
                           T &transform) {  // NOLINT
    transform =
        Eigen::Translation3f(transform_msg.translation.x,
                             transform_msg.translation.y,
                             transform_msg.translation.z) *
        Eigen::Quaternionf(transform_msg.rotation.w, transform_msg.rotation.x,
                           transform_msg.rotation.y, transform_msg.rotation.z);
  }

  PoseHistory odometry_history_;
  CloudHistory cloud_history_;

  bool getInterpolatedPose(const uint64_t &nsec,
                           Eigen::Isometry3d& pose) const;

  bool processCloud(const OusterCloud::Ptr& pointcloud,
                    const uint64_t timestamp);


};
}  // namespace lidar_undistortion

#endif  // LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_
