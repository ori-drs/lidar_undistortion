#include "lidar_undistortion/velodyne_image_converter.hpp"

using namespace lidar_undistortion;

void VelodyneImageConverter::convert(const VelodyneCloud& pc,
                                     cv::Mat& ranges,
                                     cv::Mat& altitudes,
                                     cv::Mat& azimuths) {
  for (int u = 0; u < H; u++) {
    for (int v = 0; v < W; v++) {
      const size_t index = v * H + u;
      const auto& pt = pc[index];
      Eigen::Vector3d cartesian;
      Eigen::Vector3d polar;
      cartesian << pt.x, pt.y, pt.z;
      cartesianToPolar(cartesian, polar);
      ranges.at<double>(cv::Point(v,u)) = polar(0);
      altitudes.at<double>(cv::Point(v,u)) = polar(1);
      azimuths.at<double>(cv::Point(v,u)) = polar(2);
    }
  }
}




