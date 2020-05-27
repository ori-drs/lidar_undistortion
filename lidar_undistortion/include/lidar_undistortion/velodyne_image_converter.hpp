#pragma once
#include "lidar_undistortion/lidar_image_converter.hpp"
#include <velodyne_pointcloud/rawdata.h>

namespace lidar_undistortion {
class VelodyneImageConverter : public LidarImageConverter<velodyne_pointcloud::PointXYZIR>
{
public:
  using VelodyneCloud = pcl::PointCloud<velodyne_pointcloud::PointXYZIR>;

  VelodyneImageConverter() = default;

  VelodyneImageConverter(int w, int h) : W(w), H(h) {

  }

  void convert(const VelodyneCloud& pc,
               cv::Mat& ranges,
               cv::Mat& altitudes,
               cv::Mat& azimuths) override;

private:
  int W;
  int H;
};

}



