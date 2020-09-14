#pragma once
#include "lidar_undistortion/lidar_image_converter.hpp"
#include "lidar_undistortion/ouster_point.hpp"
#include <ouster/os1_util.h>

namespace lidar_undistortion {
class OusterImageConverter : public LidarImageConverter<PointOuster> {
public:
  using OusterPoint = PointOuster;
  using OusterCloud = pcl::PointCloud<OusterPoint>;

  OusterImageConverter(int w, int h) :
    W(w), H(h), pixel_offset_(ouster::OS1::get_px_offset(W))
  {
    std::cout << "Pixel offsets:" << std::endl;

    for(auto& it : pixel_offset_){
      std::cout << it << std::endl;
    }

  }

  void convert(const OusterCloud& pc,
               cv::Mat& ranges,
               cv::Mat& altitudes,
               cv::Mat& azimuths,
               cv::Mat& intensities,
               cv::Mat& reflectivities) override;

  void cartesianToSpherical(const Eigen::Vector3d& cartesian,
                            Eigen::Vector3d& spherical) const override
  {
    // convert cartesian into ISO convention
    LidarImageConverter::cartesianToSpherical(cartesian, spherical);
    // convert from ISO to Ouster convention
    // theta ouster is azimuth
    // phi ouster is altitude
    double theta_ouster = 2*M_PI - spherical(2);
    double phi_ouster = M_PI_4 - spherical(1);
    spherical(1) = theta_ouster;
    spherical(2) = phi_ouster;
  }

  void sphericalToCartesian(const Eigen::Vector3d &spherical,
                            Eigen::Vector3d &cartesian) const override
  {
    // convert from Ouster to ISO convention
    // theta_iso is is altitude
    // phi_iso is azimuth
    double theta_iso = M_PI_4 - spherical(2);
    double phi_iso = 2*M_PI - spherical(1);
    Eigen::Vector3d spherical_iso;
    spherical_iso << spherical(0), theta_iso, phi_iso;
    // convert from ISO to cartesian
    LidarImageConverter::sphericalToCartesian(spherical_iso, cartesian);
  }

private:
  int W;
  int H;
  std::vector<int> pixel_offset_;
};

}



