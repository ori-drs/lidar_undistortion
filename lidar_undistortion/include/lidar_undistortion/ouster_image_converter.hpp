#pragma once
#include "lidar_undistortion/lidar_image_converter.hpp"
#include <ouster_ros/point_os1.h>
#include <ouster/os1_util.h>

namespace lidar_undistortion {
class OusterImageConverter : public LidarImageConverter<ouster_ros::OS1::PointOS1> {
public:
  using OusterPoint = ouster_ros::OS1::PointOS1;
  using OusterCloud = pcl::PointCloud<OusterPoint>;

  OusterImageConverter(int w, int h) : W(w), H(h), pixel_offset_(ouster::OS1::get_px_offset(W)) {

  }

  void convert(const OusterCloud& pc,
               cv::Mat& ranges,
               cv::Mat& altitudes,
               cv::Mat& azimuths) override;

private:
  int W;
  int H;
  std::vector<int> pixel_offset_;
};

}



