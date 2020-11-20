#include "lidar_undistortion/ouster_config.hpp"

using namespace lidar_undistortion;

OusterConfig::OusterConfig() {
  pixel_offsets_ = os::default_data_format(os::lidar_mode::MODE_1024x10).pixel_shift_by_row;
  lidar_to_beam_offset_mm = getLidarToBeamOffsetMilliMeter(OusterSeries::OS1_GEN1);
  DEBUG_PRINTLN("OusterConfig constructor called");
  DEBUG_PRINTLN("Pixel offsets: ");
  DEBUG_PRINTLN("lidar_to_beam_offset_mm : " << lidar_to_beam_offset_mm);
  DEBUG_PRINTLN("H : " << H);
  DEBUG_PRINTLN("W : " << W);
}

OusterConfig::OusterConfig(const os::sensor_info& info)
{
  if(info.mode != os::lidar_mode::MODE_UNSPEC){
    pixel_offsets_ = info.format.pixel_shift_by_row;
    lidar_to_beam_offset_mm = info.lidar_origin_to_beam_origin_mm;
    H = info.format.pixels_per_column;
    W = info.format.columns_per_frame;
  } else {
    ERROR_PRINTLN("Trying to set OusterConfig with unspecified lidar mode");
  }
}
