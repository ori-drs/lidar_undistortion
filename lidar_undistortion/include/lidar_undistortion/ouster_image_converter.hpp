#pragma once
#include "lidar_undistortion/lidar_image_converter.hpp"
#include "lidar_undistortion/ouster_point.hpp"
#include <ouster/types.h>



// this is a forward declaration, the implementation is
// in ouster/types.cpp (i.e., libouster_client.so)
namespace ouster {
namespace sensor {
static double default_lidar_origin_to_beam_origin(std::string prod_line);
}
}

// namespace alias
namespace os = ouster::sensor;

namespace lidar_undistortion {

enum class OusterModel {
  OS1_64_GEN1,
  OS0_128,
  OS0_64
};



class OusterConfig {

public:
  /**
   * @brief OusterConfig Default constructor for Gen1 Ouster OS1-64
   */
  OusterConfig() {
    pixel_offsets_ = os::default_data_format(os::lidar_mode::MODE_1024x10).pixel_shift_by_row;
    // NOTE: due to a possible bug in the new generation function
    // of ouster::sensor::default_data_format() the values are inverted, so we
    // reverse the vector to compensate for this.
    // For more info, compare line 71 of os1_util.cpp from the old driver code
    // and line 51 of types.cpp from the new driver code
    std::reverse(pixel_offsets_.begin(), pixel_offsets_.end());

    lidar_to_beam_offset_mm = os::default_lidar_origin_to_beam_origin("");
    H = 64;
    W = 1024;

  }
  OusterConfig(const os::sensor_info& info)
    : pixel_offsets_(info.format.pixel_shift_by_row),
      lidar_to_beam_offset_mm(info.lidar_origin_to_beam_origin_mm),
      H(info.format.pixels_per_column),
      W(info.format.columns_per_frame)
  {
  }

public:
  std::vector<int> pixel_offsets_ = os::default_data_format(os::lidar_mode::MODE_1024x10).pixel_shift_by_row;
  double lidar_to_beam_offset_mm = os::default_lidar_origin_to_beam_origin("OS-1-64");
  size_t H = 64;
  size_t W = 1024;
};



class OusterImageConverter : public LidarImageConverter<PointOuster> {
public:
  using OusterPoint = PointOuster;
  using OusterCloud = pcl::PointCloud<OusterPoint>;

  OusterImageConverter(const OusterConfig& cfg) :
    cfg_(cfg)
  {
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
  const OusterConfig& cfg_;
};

}



