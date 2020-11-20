#pragma once
#include <ouster/types.h>

namespace lidar_undistortion {

void populate_metadata_defaults(ouster::sensor::sensor_info& info,
                                ouster::sensor::lidar_mode specified_lidar_mode);

std::string read_metadata(const std::string& meta_file);
}
