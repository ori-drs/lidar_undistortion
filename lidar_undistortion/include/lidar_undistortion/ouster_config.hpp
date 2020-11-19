#pragma once
#include "lidar_undistortion/print_macros.hpp"
#include <ouster/types.h>

// namespace alias
namespace os = ouster::sensor;

namespace lidar_undistortion {

enum class OusterSeries {
  OS1_GEN1,
  OS0_GEN2,
  OS1_GEN2,
  OS2_GEN2
};

// values taken from types.cpp in ouster_client
constexpr double getLidarToBeamOffsetMilliMeter(OusterSeries series) {
  switch(series){
  case OusterSeries::OS1_GEN1:
    return 12.163;
  case OusterSeries::OS0_GEN2:
    return 27.67;
  case OusterSeries::OS1_GEN2:
    return 15.806;
  case OusterSeries::OS2_GEN2:
    return 13.762;
  default:
    return 12.163;
  }
}

class OusterConfig {

public:
  /**
   * @brief OusterConfig Default constructor for Gen1 Ouster OS1-64
   */
  OusterConfig();
  OusterConfig(const os::sensor_info& info);

  virtual ~OusterConfig() = default;

public:
  std::vector<int> pixel_offsets_;
  double lidar_to_beam_offset_mm;
  size_t H = 64;
  size_t W = 1024;

};
}
