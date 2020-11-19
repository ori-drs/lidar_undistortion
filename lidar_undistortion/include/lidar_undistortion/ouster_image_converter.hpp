#pragma once
#include "lidar_undistortion/lidar_image_converter.hpp"
#include "lidar_undistortion/ouster_point.hpp"
#include "lidar_undistortion/print_macros.hpp"
#include "lidar_undistortion/ouster_config.hpp"

namespace lidar_undistortion {

class OusterImageConverter : public LidarImageConverter<PointOuster> {
public:
  using OusterPoint = PointOuster;
  using OusterCloud = pcl::PointCloud<OusterPoint>;

  OusterImageConverter(const OusterConfig& cfg) :
    cfg_(cfg)
  {
  }

  // default constructor for OS1-64 Gen1 sensor
  OusterImageConverter() : OusterImageConverter(OusterConfig()) {
    DEBUG_PRINTLN( "H     : " << cfg_.H              );
    DEBUG_PRINTLN( "W     : " << cfg_.W              );
    DEBUG_PRINTLN( "OusterImageConverter constructor");
    DEBUG_PRINTLN( cfg_.pixel_offsets_[0]            );
  }

  ~OusterImageConverter() override {

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
  OusterConfig cfg_;
};

}



