// NOTE: code adapted from ouster driver os_node.cpp
#include "lidar_undistortion/ouster_metadata_utils.hpp"

#include <ouster/version.h>
#include <ouster/client.h>
#include <sstream>
#include <fstream>
#include "lidar_undistortion/print_macros.hpp"


using namespace ouster;

namespace lidar_undistortion {

// fill in values that could not be parsed from metadata
void populate_metadata_defaults(sensor::sensor_info& info,
                                sensor::lidar_mode specified_lidar_mode) {
    if (!info.hostname.size()) info.hostname = "UNKNOWN";

    if (!info.sn.size()) info.sn = "UNKNOWN";

    ouster::util::version v = ouster::util::version_of_string(info.fw_rev);
    if (v == ouster::util::invalid_version)
        ERROR_PRINTLN("Unknown sensor firmware version; output may not be reliable");
    else if (v < sensor::min_version)
        ERROR_PRINTLN("Firmware < " << to_string(sensor::min_version).c_str() << " not supported; output may not be reliable");

    if (!info.mode) {
        ERROR_PRINTLN(
            "Lidar mode not found in metadata; output may not be reliable");
        info.mode = specified_lidar_mode;
    }

    if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

    if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
        ERROR_PRINTLN("Beam angles not found in metadata; using design values");
        info.beam_azimuth_angles = sensor::gen1_azimuth_angles;
        info.beam_altitude_angles = sensor::gen1_altitude_angles;
    }

    if (info.imu_to_sensor_transform.empty() ||
        info.lidar_to_sensor_transform.empty()) {
        ERROR_PRINTLN("Frame transforms not found in metadata; using design values");
        info.imu_to_sensor_transform = sensor::imu_to_sensor_transform;
        info.lidar_to_sensor_transform = sensor::lidar_to_sensor_transform;
    }
}

// try to read metadata file
std::string read_metadata(const std::string& meta_file) {
    if (meta_file.size()) {
        ERROR_PRINTLN("Reading metadata from " << meta_file.c_str());
    } else {
        ERROR_PRINTLN("No metadata file specified");
        return "";
    }

    std::stringstream buf{};
    std::ifstream ifs{};
    ifs.open(meta_file);
    buf << ifs.rdbuf();
    ifs.close();

    if (!ifs)
        ERROR_PRINTLN("Failed to read " << meta_file.c_str() << " check that the path is valid");

    return buf.str();
}

}
