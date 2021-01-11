// NOTE: code adapted from ouster driver os_node.cpp
#include "lidar_undistortion/ouster_metadata_utils.hpp"

#include <sstream>
#include <fstream>
#include <json/json.h>
#include <algorithm>
#include "lidar_undistortion/print_macros.hpp"

using namespace std;

namespace ouster {
namespace util {
version version_of_string(const std::string& s) {
    std::istringstream is{s};
    char c1, c2, c3;
    version v;

    is >> c1 >> v.major >> c2 >> v.minor >> c3 >> v.patch;

    if (is && c1 == 'v' && c2 == '.' && c3 == '.')
        return v;
    else
        return invalid_version;
}

std::string to_string(const version& v) {
    if (v == invalid_version) return "UNKNOWN";

    std::stringstream ss{};
    ss << "v" << v.major << "." << v.minor << "." << v.patch;
    return ss.str();
}

} // namespace util

namespace sensor {
data_format default_data_format(lidar_mode mode) {
    auto repeat = [](int n, const std::vector<int>& v) {
        std::vector<int> res{};
        for (int i = 0; i < n; i++) res.insert(res.end(), v.begin(), v.end());
        return res;
    };

    uint32_t pixels_per_column = 64;
    uint32_t columns_per_packet = 16;
    uint32_t columns_per_frame = n_cols_of_lidar_mode(mode);

    std::vector<int> offset;
    switch (columns_per_frame) {
        case 512:
            offset = repeat(16, {9, 6, 3, 0});
            break;
        case 1024:
            offset = repeat(16, {18, 12, 6, 0});
            break;
        case 2048:
            offset = repeat(16, {36, 24, 12, 0});
            break;
        default:
            offset = repeat(16, {18, 12, 6, 0});
            break;
    }

    return {pixels_per_column, columns_per_packet, columns_per_frame, offset};
}

int n_cols_of_lidar_mode(lidar_mode mode) {
    switch (mode) {
        case MODE_512x10:
        case MODE_512x20:
            return 512;
        case MODE_1024x10:
        case MODE_1024x20:
            return 1024;
        case MODE_2048x10:
            return 2048;
        default:
            throw std::invalid_argument{"n_cols_of_lidar_mode"};
    }
}

sensor_info parse_metadata(const std::string& meta) {
    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{meta};

    if (meta.size()) {
        if (!Json::parseFromStream(builder, ss, &root, &errors))
            throw std::runtime_error{errors.c_str()};
    }

    sensor_info info{};

    info.hostname = root["hostname"].asString();
    info.sn = root["prod_sn"].asString();
    info.fw_rev = root["build_rev"].asString();
    info.mode = lidar_mode_of_string(root["lidar_mode"].asString());
    info.prod_line = root["prod_line"].asString();

    // "data_format" introduced in fw 1.14. Fall back to common 1.13 parameters
    // otherwise
    if (root.isMember("data_format")) {
        info.format.pixels_per_column =
            root["data_format"]["pixels_per_column"].asInt();
        info.format.columns_per_packet =
            root["data_format"]["columns_per_packet"].asInt();
        info.format.columns_per_frame =
            root["data_format"]["columns_per_frame"].asInt();

        for (const auto& v : root["data_format"]["pixel_shift_by_row"])
            info.format.pixel_shift_by_row.push_back(v.asInt());
    } else {
        info.format = default_data_format(info.mode);
    }

    // "lidar_origin_to_beam_origin_mm" introduced in fw 1.14. Fall back to
    // common 1.13 parameters otherwise
    if (root.isMember("lidar_origin_to_beam_origin_mm")) {
        info.lidar_origin_to_beam_origin_mm =
            root["lidar_origin_to_beam_origin_mm"].asDouble();
    } else {
        info.lidar_origin_to_beam_origin_mm =
            default_lidar_origin_to_beam_origin(info.prod_line);
    }

    for (const auto& v : root["beam_altitude_angles"])
        info.beam_altitude_angles.push_back(v.asDouble());

    for (const auto& v : root["beam_azimuth_angles"])
        info.beam_azimuth_angles.push_back(v.asDouble());

    for (const auto& v : root["imu_to_sensor_transform"])
        info.imu_to_sensor_transform.push_back(v.asDouble());

    for (const auto& v : root["lidar_to_sensor_transform"])
        info.lidar_to_sensor_transform.push_back(v.asDouble());

    return info;
}

lidar_mode lidar_mode_of_string(const std::string& s) {
    auto end = lidar_mode_strings.end();
    auto res = std::find_if(lidar_mode_strings.begin(), end,
                            [&](const std::pair<lidar_mode, std::string>& p) {
                                return p.second == s;
                            });

    return res == end ? lidar_mode(0) : res->first;
}

double default_lidar_origin_to_beam_origin(std::string prod_line) {
    double lidar_origin_to_beam_origin_mm = 12.163;  // default for gen 1
    if (prod_line.find("OS-0-") == 0)
        lidar_origin_to_beam_origin_mm = 27.67;
    else if (prod_line.find("OS-1-") == 0)
        lidar_origin_to_beam_origin_mm = 15.806;
    else if (prod_line.find("OS-2-") == 0)
        lidar_origin_to_beam_origin_mm = 13.762;
    return lidar_origin_to_beam_origin_mm;
}

} // namespace sensor
} // namespace ouster

using namespace ouster;

namespace lidar_undistortion {

// fill in values that could not be parsed from metadata
void populate_metadata_defaults(sensor::sensor_info& info,
                                sensor::lidar_mode specified_lidar_mode) {
    if (!info.hostname.size()) info.hostname = "UNKNOWN";

    if (!info.sn.size()) info.sn = "UNKNOWN";

    ouster::util::version v = ouster::util::version_of_string(info.fw_rev);
    if (v == ouster::util::invalid_version)
        INFO_PRINTLN("Unknown sensor firmware version; output may not be reliable");
    else if (v < sensor::min_version)
        ERROR_PRINTLN("Firmware < " << to_string(sensor::min_version).c_str() << " not supported; output may not be reliable");

    if (!info.mode) {
        ERROR_PRINTLN("Lidar mode not found in metadata; output may not be reliable");
        info.mode = specified_lidar_mode;
    }

    if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

    if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
        INFO_PRINTLN("Beam angles not found in metadata; using design values");
        info.beam_azimuth_angles = sensor::gen1_azimuth_angles;
        info.beam_altitude_angles = sensor::gen1_altitude_angles;
    }

    if (info.imu_to_sensor_transform.empty() ||
        info.lidar_to_sensor_transform.empty()) {
        INFO_PRINTLN("Frame transforms not found in metadata; using design values");
        info.imu_to_sensor_transform = sensor::imu_to_sensor_transform;
        info.lidar_to_sensor_transform = sensor::lidar_to_sensor_transform;
    }
}

// try to read metadata file
std::string read_metadata(const std::string& meta_file) {
    if (meta_file.size()) {
        INFO_PRINTLN("Reading metadata from " << meta_file.c_str());
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
