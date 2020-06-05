#include "lidar_undistortion/velodyne_image_converter.hpp"

using namespace lidar_undistortion;

void VelodyneImageConverter::convert(const VelodyneCloud& pc,
                                     cv::Mat& ranges,
                                     cv::Mat& altitudes,
                                     cv::Mat& azimuths,
                                     cv::Mat& intensities,
                                     cv::Mat& reflectivities)
{
  //std::cerr << "H = " << H << " W = " << W << std::endl;

  for (size_t u = 0; u < H; u++) {
    for (size_t v = 0; v < W; v++) {
      const size_t vv = (v + pivot_offset+1) % W;
      const size_t index = u * W + vv;

      const auto& pt = pc[index];
      Eigen::Vector3d cartesian;
      Eigen::Vector3d polar;
      cartesian << pt.x, pt.y, pt.z;
      cartesianToPolar(cartesian, polar);
      // std::cerr << "[" << v << ", " << u  << "] " << std::endl;
        //std::cerr << ranges.cols << " " << ranges.rows << std::endl;
        //std::cerr << altitudes.cols << " " << altitudes.rows << std::endl;
        //
        //std::cerr << azimuths.cols << " " << azimuths.rows << std::endl;

        ranges.at<double>(cv::Point(v, u*8 + 0)) = polar(0);
        ranges.at<double>(cv::Point(v, u*8 + 1)) = polar(0);
        ranges.at<double>(cv::Point(v, u*8 + 2)) = polar(0);
        ranges.at<double>(cv::Point(v, u*8 + 3)) = polar(0);
        ranges.at<double>(cv::Point(v, u*8 + 4)) = polar(0);
        ranges.at<double>(cv::Point(v, u*8 + 5)) = polar(0);
        ranges.at<double>(cv::Point(v, u*8 + 6)) = polar(0);
        ranges.at<double>(cv::Point(v, u*8 + 7)) = polar(0);

        // altitudes.at<double>(cv::Point(v, u*8 + i)) = polar(1);
        // azimuths.at<double>(cv::Point(v, u*8 + i)) = polar(2);


    }
  }
}




