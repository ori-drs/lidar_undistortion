#include "lidar_undistortion/ouster_image_converter.hpp"

using namespace lidar_undistortion;

void OusterImageConverter::convert(const OusterCloud& pc,
                                   cv::Mat& ranges,
                                   cv::Mat& altitudes,
                                   cv::Mat& azimuths,
                                   cv::Mat& intensities,
                                   cv::Mat& reflectivities)
{
  for (std::size_t u = 0; u < cfg_.H; u++) {
    for (std::size_t v = 0; v < cfg_.W; v++) {
      const size_t vv = (v + cfg_.W - cfg_.pixel_offsets_[u]) % cfg_.W;
      const size_t index = u * cfg_.W + vv;
      DEBUG_PRINTLN("u     : " << u                     );
      DEBUG_PRINTLN("v     : " << v                     );
      DEBUG_PRINTLN("vv    : " << vv                    );
      DEBUG_PRINTLN("index : " << index                 );
      DEBUG_PRINTLN("offset: " << cfg_.pixel_offsets_[u]);

      const auto& pt = pc[index];      
      Eigen::Vector3d cartesian;
      Eigen::Vector3d spherical;
      cartesian << pt.x, pt.y, pt.z;
      cartesianToSpherical(cartesian, spherical);
      ranges.at<double>(cv::Point(v,u)) = spherical(0);
      azimuths.at<double>(cv::Point(v,u)) = spherical(1);
      altitudes.at<double>(cv::Point(v,u)) = spherical(2);
      intensities.at<double>(cv::Point(v,u)) = pt.intensity;
      reflectivities.at<double>(cv::Point(v,u)) = pt.reflectivity;
    }
  }
}


